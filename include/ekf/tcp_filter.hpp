/*
 * tcp_filter.hpp
 *
 * Standalone replacement for robot_localization's ros_filter.hpp.
 *
 * This is the same predict/correct EKF pipeline as the original RosFilter,
 * but instead of subscribing to ROS topics (sensor_msgs/Imu,
 * nav_msgs/Odometry) it reads the same IMU/Odom data as raw structs over a
 * plain TCP socket, using the wire format sent by server.c (see header.hpp
 * and tcp_protocol.hpp).
 *
 * What was intentionally dropped from the original, and why:
 *
 *  - rclcpp::Node / topics / services / dynamic reconfigure / diagnostics:
 *    there is no ROS here, so all of this is gone. Configuration that used
 *    to come from a YAML param file is now a handful of member variables
 *    you set directly in main() (see tcp_filter.cpp) before calling run().
 *
 *  - tf2 transforms: the original used tf2 to transform each sensor
 *    reading into a common target frame (and to rotate covariances)
 *    before fusing it. There is no tf tree here, so every measurement is
 *    fused as-is, in the frame it was reported in. This is fine as long as
 *    your IMU and Odom sources are already expressed in the same frame
 *    convention (which is the case for the imu_link / odom / base_link
 *    setup produced by server.c). If you need real multi-frame support you
 *    will need to bring back a transform library.
 *
 *  - Differential / relative pose integration, multiple sensors per type,
 *    GPS, control input: server.c only ever sends one IMU stream and one
 *    Odom stream, so those code paths (which existed to support many
 *    simultaneously-configured sensors) are not reproduced. The
 *    CallbackData concept is kept, but only two fixed instances of it are
 *    used (one for IMU, one for Odom).
 *
 * Everything else - the measurement queue, the lagged-smoothing history,
 * the EKF predict/correct cycle itself (via FilterBase/Ekf) - mirrors the
 * original RosFilter as closely as possible.
 */

#ifndef TCP_FILTER_HPP_
#define TCP_FILTER_HPP_

#include <Eigen/Dense>

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

// Adjust these include paths to match wherever your converted
// ekf.hpp / filter_base.hpp / filter_common.hpp / filter_time.hpp /
// measurement.hpp / filter_state.hpp actually live.
#include "ekf/filter_base.hpp"
#include "ekf/filter_common.hpp"
#include "filter_time.hpp"
#include "ekf/filter_state.hpp"
#include "ekf/measurement.hpp"

#include "header.hpp"
#include "tcp_protocol.hpp"

namespace robot_localization
{

//! @brief Per-sensor-channel configuration (which state variables this
//! channel is allowed to update, and its outlier-rejection threshold).
//!
//! This is a trimmed-down version of the original CallbackData: it drops
//! the differential_/relative_ flags because server.c's Odom stream is
//! already an absolute pose, not something we need to integrate
//! differentially.
//!
struct CallbackData
{
  CallbackData(
    const std::string & topic_name,
    const std::vector<bool> & update_vector,
    const double rejection_threshold)
  : topic_name_(topic_name), update_vector_(update_vector),
    rejection_threshold_(rejection_threshold)
  {
    update_sum_ = 0;
    for (size_t i = 0; i < update_vector_.size(); ++i) {
      update_sum_ += update_vector_[i] ? 1 : 0;
    }
  }

  std::string topic_name_;
  std::vector<bool> update_vector_;
  int update_sum_;
  double rejection_threshold_;
};

using MeasurementQueue =
  std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
    Measurement>;
using MeasurementHistoryDeque = std::deque<MeasurementPtr>;
using FilterStateHistoryDeque = std::deque<FilterStatePtr>;

//! @brief Result of the fused state, in plain (non-ROS) form.
//!
//! This is what getFilteredOdometry() fills in - a local stand-in for the
//! nav_msgs::msg::Odometry the original code published.
//!
struct FilteredState
{
  double stamp_sec;
  std::string frame_id;
  std::string child_frame_id;

  double x, y, z;      //position
  double qx, qy, qz, qw;   // orientation, as a quaternion(in yaml these as x,y,z,w)
  double roll, pitch, yaw; // orientation, as Euler angles (for convenience/logging)

  double vx, vy, vz;   //linear velocity
  double vroll, vpitch, vyaw; //angular velocity

  double pose_covariance[36];
  double twist_covariance[36];
};

template<class T>
class TcpFilter
{
public:
  //! @brief Constructor.
  //! @param[in] server_ip - IP address of the TCP server (see server.c)
  //! @param[in] port - TCP port of the server
  //!
  TcpFilter(const std::string & server_ip, int port);

  ~TcpFilter();

  //! @brief Connects to the TCP server. Must be called before run().
  //! @return true on success
  //!
  bool connectToServer();

  //! @brief Main loop: reads packets from the socket, enqueues
  //! measurements, and periodically drives the EKF forward, forever (or
  //! until the connection drops).
  //!
  void run();

  //! @brief Resets the filter to its initial state.
  //!
  void reset();

  //! @brief Access to the underlying filter (Ekf, Ukf, ...).
  //!
  T & getFilter() {return filter_;}

  // ---- Configuration (set these before calling run()) -------------------

  //! @brief If true, Z/roll/pitch/Vz/Vroll/Vpitch/Az are locked to 0.
  bool two_d_mode_;

  //! @brief If true, once a sensor timeout elapses with no new
  //! measurements, keep predicting forward to "now" anyway.
  bool predict_to_current_time_;

  //! @brief If true, subtract gravity from the IMU's linear_acceleration
  //! before fusing it, using the filter's current orientation estimate.
  bool remove_gravitational_acceleration_;

  //! @brief Gravitational acceleration constant (m/s^2), used only if
  //! remove_gravitational_acceleration_ is true.
  double gravitational_acceleration_;

  //! @brief Rate, in Hz, at which periodicUpdate() runs.
  double frequency_;

  //! @brief If true, keep a bounded history of filter states/measurements
  //! so that a late-arriving (out-of-order) measurement can be smoothed
  //! in by reverting and replaying, instead of just being dropped.
  bool smooth_lagged_data_;

  //! @brief How much history to retain when smooth_lagged_data_ is true.
  Duration history_length_;

  //! @brief Frame id written into the fused output.
  std::string world_frame_id_;

  //! @brief Child frame id written into the fused output.
  std::string base_link_output_frame_id_;

  //! @brief If true, print the fused state to stdout every periodicUpdate().
  bool print_output_;

  //! @brief If true, send the fused state back over the TCP socket as a
  //! PACKET_FILTERED_ODOM packet every periodicUpdate().
  bool send_filtered_output_;

  //! @brief Per-channel configuration for the two data sources server.c
  //! sends. Populate these in main() before calling run(). Sizes must be
  //! STATE_SIZE (15).
  std::vector<bool> imu_pose_config_;    // roll, pitch, yaw from IMU orientation
  std::vector<bool> imu_twist_config_;   // angular velocity from IMU
  std::vector<bool> imu_accel_config_;   // linear acceleration from IMU
  std::vector<bool> odom_pose_config_;   // x,y,z,roll,pitch,yaw from Odom
  std::vector<bool> odom_twist_config_;  // vx,vy,vz,vroll,vpitch,vyaw from Odom

  double imu_mahalanobis_thresh_;
  double odom_mahalanobis_thresh_;

protected:
  // ---- Packet handling ----------------------------------------------

  bool readExact(void * buf, size_t len);
  bool readOnePacket();

  void handleImuPacket(const ImuData & imu);
  void handleOdomPacket(const OdomData & odom);

  // ---- Measurement preparation (tf2-free versions of the originals) -

  bool prepareImuPose(
    const ImuData & imu, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  bool prepareImuTwist(
    const ImuData & imu, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  bool prepareAcceleration(
    const ImuData & imu, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  bool prepareOdomPose(
    const OdomData & odom, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  bool prepareOdomTwist(
    const OdomData & odom, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  void forceTwoD(
    Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  // ---- Filter driving --------------------------------------------------

  void enqueueMeasurement(
    const std::string & topic_name,
    const Eigen::VectorXd & measurement,
    const Eigen::MatrixXd & measurement_covariance,
    const std::vector<bool> & update_vector,
    const double mahalanobis_thresh,
    const Time & time);

  void integrateMeasurements(const Time & current_time);

  void periodicUpdate();

  bool getFilteredOdometry(FilteredState & out);

  bool validateFilterOutput(const FilteredState & state);

  void printFilteredState(const FilteredState & state);

  void sendFilteredOdom(const FilteredState & state);

  // ---- Lagged-smoothing history (mirrors the original) ------------------

  void saveFilterState(T & filter);
  bool revertTo(const Time & time);
  void clearExpiredHistory(const Time & cutoff_time);
  void clearMeasurementQueue();

  // ---- Small math helpers (replace tf2::Quaternion / tf2::Matrix3x3) ----

  static void quaternionToRPY(
    double x, double y, double z, double w,
    double & roll, double & pitch, double & yaw);

  static void rpyToQuaternion(
    double roll, double pitch, double yaw,
    double & x, double & y, double & z, double & w);

  //! @brief Fills R with the body->world rotation matrix for the given
  //! roll/pitch/yaw (ZYX convention, same as tf2::Matrix3x3::setRPY).
  static void rpyToRotationMatrix(
    double roll, double pitch, double yaw, double R[3][3]);

  // ---- State ----------------------------------------------------------

  std::string server_ip_;
  int port_;
  int sockfd_;

  Time last_set_pose_time_;
  Time last_published_stamp_;

  std::map<std::string, Time> last_message_times_;

  MeasurementQueue measurement_queue_;
  MeasurementHistoryDeque measurement_history_;
  FilterStateHistoryDeque filter_state_history_;

  T filter_;
};

}  // namespace robot_localization

#endif  // TCP_FILTER_HPP_

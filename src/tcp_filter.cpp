/*
 * tcp_filter.cpp
 *
 * Standalone replacement for robot_localization's ros_filter.cpp.
 * See tcp_filter.hpp for a summary of what was dropped vs. the original
 * (tf2 transforms, ROS params/topics/services/diagnostics, multi-sensor
 * support) and why.
 *
 * Build (adjust include/library paths for your project layout):
 *
 *   g++ -std=c++17 -O2 \
 *       -I. -I/usr/include/eigen3 \
 *       tcp_filter.cpp ekf.cpp filter_base.cpp filter_utilities.cpp \
 *       -o tcp_filter
 *
 * Run:
 *
 *   ./tcp_filter [server_ip] [port]
 *
 * with server.c already running and listening.
 */

#include "ekf/tcp_filter.hpp"
#include "ekf/ekf.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include<ctime>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
//! @brief Wall-clock seconds since the Unix epoch.
//!
//! This has to be epoch-based (not steady_clock) because the packet
//! timestamps coming from server.c (header.stamp.sec/nanosec) are
//! themselves Unix timestamps - the filter compares "now" against
//! measurement times directly, so both need to be on the same clock.
//!
double nowSeconds()
{
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

namespace robot_localization
{

// ============================================================================
// Construction / destruction
// ============================================================================

template<class T>
TcpFilter<T>::TcpFilter(const std::string & server_ip, int port)
: server_ip_(server_ip), port_(port), sockfd_(-1)
{
  two_d_mode_ = false;
  predict_to_current_time_ = true;
  remove_gravitational_acceleration_ = false;
  gravitational_acceleration_ = 9.80665;
  frequency_ = 10.0;
  smooth_lagged_data_ = false;
  history_length_ = Duration(0.0);
  world_frame_id_ = "odom";
  base_link_output_frame_id_ = "base_link";
  print_output_ = true;
  send_filtered_output_ = false;
  imu_mahalanobis_thresh_ = std::numeric_limits<double>::max();
  odom_mahalanobis_thresh_ = std::numeric_limits<double>::max();

  imu_pose_config_ = std::vector<bool>(STATE_SIZE, false);
  imu_twist_config_ = std::vector<bool>(STATE_SIZE, false);
  imu_accel_config_ = std::vector<bool>(STATE_SIZE, false);
  odom_pose_config_ = std::vector<bool>(STATE_SIZE, false);
  odom_twist_config_ = std::vector<bool>(STATE_SIZE, false);

  last_set_pose_time_ = Time(0.0);
  last_published_stamp_ = Time(0.0);

  // Reasonable general-purpose defaults - tune these for your platform.
  // (These replace the process_noise_covariance / initial_estimate_covariance
  // YAML parameters the original node loaded.)
  Eigen::MatrixXd process_noise_covariance =
    Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.05;
  filter_.setProcessNoiseCovariance(process_noise_covariance);

  Eigen::MatrixXd initial_estimate_error_covariance =
    Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 1.0;
  filter_.setEstimateErrorCovariance(initial_estimate_error_covariance);

  filter_.setSensorTimeout(Duration(1.0));
}

template<class T>
TcpFilter<T>::~TcpFilter()
{
  if (sockfd_ >= 0) {
    close(sockfd_);
  }
}

template<class T>
void TcpFilter<T>::reset()
{
  filter_.reset();
  clearMeasurementQueue();
  measurement_history_.clear();
  filter_state_history_.clear();
  last_message_times_.clear();
  last_set_pose_time_ = Time(0.0);
  last_published_stamp_ = Time(0.0);
}

// ============================================================================
// Networking
// ============================================================================

template<class T>
bool TcpFilter<T>::connectToServer()
{
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0) {
    perror("socket");
    return false;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (inet_pton(AF_INET, server_ip_.c_str(), &addr.sin_addr) <= 0) {
    std::cerr << "Invalid server IP: " << server_ip_ << "\n";
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  std::cout << "Connecting to " << server_ip_ << ":" << port_ << " ...\n";

  if (connect(sockfd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("connect");
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  std::cout << "Connected.\n";
  return true;
}

template<class T>
bool TcpFilter<T>::readExact(void * buf, size_t len)
{
  uint8_t * p = static_cast<uint8_t *>(buf);
  size_t got = 0;
  while (got < len) {
    ssize_t n = recv(sockfd_, p + got, len - got, 0);
    std::cout << "recv returned = " << n << std::endl;
    if (n <= 0) {
      return false;  // connection closed, or a real error
    }
    got += static_cast<size_t>(n);
  }
  return true;
}

template<class T>
bool TcpFilter<T>::readOnePacket()
{
  PacketHeader hdr;
  if (!readExact(&hdr, sizeof(hdr))) {
    return false;
  }

  if (hdr.type == PACKET_IMU) {
    ImuData imu;
    if (!readExact(&imu, sizeof(imu))) {
      return false;
    }
    handleImuPacket(imu);
  } else if (hdr.type == PACKET_ODOM) {
    OdomData odom;
    if (!readExact(&odom, sizeof(odom))) {
      return false;
    }
    handleOdomPacket(odom);
  } else {
    std::cerr << "Unknown packet type (" << hdr.type <<
      "). Dropping connection.\n";
    return false;
  }

  return true;
}

// ============================================================================
// Packet -> measurement handling
// (replaces imuCallback()/odometryCallback() from the original)
// ============================================================================

namespace
{
bool hasAnyUpdate(const std::vector<bool> & v)
{
  for (size_t i = 0; i < v.size(); ++i) {
    if (v[i]) {return true;}
  }
  return false;
}
}  // namespace

template<class T>
void TcpFilter<T>::handleImuPacket(const ImuData & imu)
{
  Time msg_time(
    static_cast<double>(imu.header.stamp.sec) +
    static_cast<double>(imu.header.stamp.nanosec) * 1e-9);

  // If we've just reset the filter, ignore anything at or before that time.
  if (last_set_pose_time_ >= msg_time) {
    return;
  }

  if (imu_pose_config_.size() == static_cast<size_t>(STATE_SIZE) &&
    hasAnyUpdate(imu_pose_config_))
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    std::vector<bool> update_vector = imu_pose_config_;
    if (prepareImuPose(imu, measurement, covariance, update_vector)) {
      enqueueMeasurement(
        "imu_pose", measurement, covariance, update_vector,
        imu_mahalanobis_thresh_, msg_time);
    }
  }

  if (imu_twist_config_.size() == static_cast<size_t>(STATE_SIZE) &&
    hasAnyUpdate(imu_twist_config_))
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    std::vector<bool> update_vector = imu_twist_config_;
    if (prepareImuTwist(imu, measurement, covariance, update_vector)) {
      enqueueMeasurement(
        "imu_twist", measurement, covariance, update_vector,
        imu_mahalanobis_thresh_, msg_time);
    }
  }

  if (imu_accel_config_.size() == static_cast<size_t>(STATE_SIZE) &&
    hasAnyUpdate(imu_accel_config_))
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    std::vector<bool> update_vector = imu_accel_config_;
    if (prepareAcceleration(imu, measurement, covariance, update_vector)) {
      enqueueMeasurement(
        "imu_accel", measurement, covariance, update_vector,
        imu_mahalanobis_thresh_, msg_time);
    }
  }

  last_message_times_["imu"] = msg_time;
}

template<class T>
void TcpFilter<T>::handleOdomPacket(const OdomData & odom)
{
  Time msg_time(
    static_cast<double>(odom.header.stamp.sec) +
    static_cast<double>(odom.header.stamp.nanosec) * 1e-9);

  if (last_set_pose_time_ >= msg_time) {
    return;
  }

  if (odom_pose_config_.size() == static_cast<size_t>(STATE_SIZE) &&
    hasAnyUpdate(odom_pose_config_))
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    std::vector<bool> update_vector = odom_pose_config_;
    if (prepareOdomPose(odom, measurement, covariance, update_vector)) {
      enqueueMeasurement(
        "odom_pose", measurement, covariance, update_vector,
        odom_mahalanobis_thresh_, msg_time);
    }
  }

  if (odom_twist_config_.size() == static_cast<size_t>(STATE_SIZE) &&
    hasAnyUpdate(odom_twist_config_))
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    std::vector<bool> update_vector = odom_twist_config_;
    if (prepareOdomTwist(odom, measurement, covariance, update_vector)) {
      enqueueMeasurement(
        "odom_twist", measurement, covariance, update_vector,
        odom_mahalanobis_thresh_, msg_time);
    }
  }

  last_message_times_["odom"] = msg_time;
}

// ============================================================================
// Measurement preparation
// (tf2-free versions of prepareAcceleration()/preparePose()/prepareTwist())
// ============================================================================

template<class T>
bool TcpFilter<T>::prepareImuPose(
  const ImuData & imu, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  // Per the sensor_msgs/Imu convention: a first covariance value of -1
  // means "no orientation data available", so skip it.
  if (std::fabs(imu.orientation_covariance[0] + 1.0) < 1e-9) {
    return false;
  }

  double roll, pitch, yaw;
  quaternionToRPY(
    imu.orientation.x, imu.orientation.y, imu.orientation.z,
    imu.orientation.w, roll, pitch, yaw);

  measurement(StateMemberRoll) = roll;
  measurement(StateMemberPitch) = pitch;
  measurement(StateMemberYaw) = yaw;

  for (int i = 0; i < ORIENTATION_SIZE; ++i) {
    for (int j = 0; j < ORIENTATION_SIZE; ++j) {
      measurement_covariance(ORIENTATION_OFFSET + i, ORIENTATION_OFFSET + j) =
        imu.orientation_covariance[ORIENTATION_SIZE * i + j];
    }
  }

  if (two_d_mode_) {
    forceTwoD(measurement, measurement_covariance, update_vector);
  }

  return true;
}

template<class T>
bool TcpFilter<T>::prepareImuTwist(
  const ImuData & imu, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  if (std::fabs(imu.angular_velocity_covariance[0] + 1.0) < 1e-9) {
    return false;
  }

  measurement(StateMemberVroll) = imu.angular_velocity.x;
  measurement(StateMemberVpitch) = imu.angular_velocity.y;
  measurement(StateMemberVyaw) = imu.angular_velocity.z;

  for (int i = 0; i < ORIENTATION_SIZE; ++i) {
    for (int j = 0; j < ORIENTATION_SIZE; ++j) {
      measurement_covariance(ORIENTATION_V_OFFSET + i, ORIENTATION_V_OFFSET + j) =
        imu.angular_velocity_covariance[ORIENTATION_SIZE * i + j];
    }
  }

  if (two_d_mode_) {
    forceTwoD(measurement, measurement_covariance, update_vector);
  }

  return true;
}

template<class T>
bool TcpFilter<T>::prepareAcceleration(
  const ImuData & imu, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  if (std::fabs(imu.linear_acceleration_covariance[0] + 1.0) < 1e-9) {
    return false;
  }

  double ax = imu.linear_acceleration.x;
  double ay = imu.linear_acceleration.y;
  double az = imu.linear_acceleration.z;

  if (remove_gravitational_acceleration_) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    const bool have_orientation =
      std::fabs(imu.orientation_covariance[0] + 1.0) >= 1e-9;

    if (have_orientation) {
      quaternionToRPY(
        imu.orientation.x, imu.orientation.y, imu.orientation.z,
        imu.orientation.w, roll, pitch, yaw);
    } else if (filter_.getInitializedStatus()) {
      // No orientation on this message - fall back to the filter's own
      // current orientation estimate, same as the original code did.
      const Eigen::VectorXd & state = filter_.getState();
      roll = state(StateMemberRoll);
      pitch = state(StateMemberPitch);
      yaw = state(StateMemberYaw);
    }

    double R[3][3];
    rpyToRotationMatrix(roll, pitch, yaw, R);

    // Gravity in the world frame is (0, 0, g). Rotate it into the body
    // frame with R^T (R is body->world) and subtract it from the raw
    // accelerometer reading.
    const double gx = R[2][0] * gravitational_acceleration_;
    const double gy = R[2][1] * gravitational_acceleration_;
    const double gz = R[2][2] * gravitational_acceleration_;

    ax -= gx;
    ay -= gy;
    az -= gz;
  }

  measurement(StateMemberAx) = ax;
  measurement(StateMemberAy) = ay;
  measurement(StateMemberAz) = az;

  for (int i = 0; i < ACCELERATION_SIZE; ++i) {
    for (int j = 0; j < ACCELERATION_SIZE; ++j) {
      measurement_covariance(POSITION_A_OFFSET + i, POSITION_A_OFFSET + j) =
        imu.linear_acceleration_covariance[ACCELERATION_SIZE * i + j];
    }
  }

  if (two_d_mode_) {
    forceTwoD(measurement, measurement_covariance, update_vector);
  }

  return true;
}

template<class T>
bool TcpFilter<T>::prepareOdomPose(
  const OdomData & odom, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  double roll, pitch, yaw;
  quaternionToRPY(
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w,
    roll, pitch, yaw);

  measurement(StateMemberX) = odom.pose.pose.position.x;
  measurement(StateMemberY) = odom.pose.pose.position.y;
  measurement(StateMemberZ) = odom.pose.pose.position.z;
  measurement(StateMemberRoll) = roll;
  measurement(StateMemberPitch) = pitch;
  measurement(StateMemberYaw) = yaw;

  // OdomData's 6x6 pose covariance is already laid out as
  // [x,y,z,roll,pitch,yaw] x [x,y,z,roll,pitch,yaw], which lines up
  // exactly with POSITION_OFFSET..(POSITION_OFFSET+POSE_SIZE-1) in the
  // state vector, so this is a direct copy (no splitting needed, unlike
  // the IMU case above).
  for (int i = 0; i < POSE_SIZE; ++i) {
    for (int j = 0; j < POSE_SIZE; ++j) {
      measurement_covariance(POSITION_OFFSET + i, POSITION_OFFSET + j) =
        odom.pose.covariance[POSE_SIZE * i + j];
    }
  }

  if (two_d_mode_) {
    forceTwoD(measurement, measurement_covariance, update_vector);
  }

  return true;
}

template<class T>
bool TcpFilter<T>::prepareOdomTwist(
  const OdomData & odom, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  measurement(StateMemberVx) = odom.twist.twist.linear.x;
  measurement(StateMemberVy) = odom.twist.twist.linear.y;
  measurement(StateMemberVz) = odom.twist.twist.linear.z;
  measurement(StateMemberVroll) = odom.twist.twist.angular.x;
  measurement(StateMemberVpitch) = odom.twist.twist.angular.y;
  measurement(StateMemberVyaw) = odom.twist.twist.angular.z;

  for (int i = 0; i < TWIST_SIZE; ++i) {
    for (int j = 0; j < TWIST_SIZE; ++j) {
      measurement_covariance(POSITION_V_OFFSET + i, POSITION_V_OFFSET + j) =
        odom.twist.covariance[TWIST_SIZE * i + j];
    }
  }

  if (two_d_mode_) {
    forceTwoD(measurement, measurement_covariance, update_vector);
  }

  return true;
}

template<class T>
void TcpFilter<T>::forceTwoD(
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  measurement(StateMemberZ) = 0.0;
  measurement(StateMemberRoll) = 0.0;
  measurement(StateMemberPitch) = 0.0;
  measurement(StateMemberVz) = 0.0;
  measurement(StateMemberVroll) = 0.0;
  measurement(StateMemberVpitch) = 0.0;
  measurement(StateMemberAz) = 0.0;

  measurement_covariance.col(StateMemberZ).setZero();
  measurement_covariance.col(StateMemberRoll).setZero();
  measurement_covariance.col(StateMemberPitch).setZero();
  measurement_covariance.col(StateMemberVz).setZero();
  measurement_covariance.col(StateMemberVroll).setZero();
  measurement_covariance.col(StateMemberVpitch).setZero();
  measurement_covariance.col(StateMemberAz).setZero();

  measurement_covariance.row(StateMemberZ).setZero();
  measurement_covariance.row(StateMemberRoll).setZero();
  measurement_covariance.row(StateMemberPitch).setZero();
  measurement_covariance.row(StateMemberVz).setZero();
  measurement_covariance.row(StateMemberVroll).setZero();
  measurement_covariance.row(StateMemberVpitch).setZero();
  measurement_covariance.row(StateMemberAz).setZero();

  measurement_covariance(StateMemberZ, StateMemberZ) = 1e-6;
  measurement_covariance(StateMemberRoll, StateMemberRoll) = 1e-6;
  measurement_covariance(StateMemberPitch, StateMemberPitch) = 1e-6;
  measurement_covariance(StateMemberVz, StateMemberVz) = 1e-6;
  measurement_covariance(StateMemberVroll, StateMemberVroll) = 1e-6;
  measurement_covariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
  measurement_covariance(StateMemberAz, StateMemberAz) = 1e-6;

  update_vector[StateMemberZ] = true;
  update_vector[StateMemberRoll] = true;
  update_vector[StateMemberPitch] = true;
  update_vector[StateMemberVz] = true;
  update_vector[StateMemberVroll] = true;
  update_vector[StateMemberVpitch] = true;
  update_vector[StateMemberAz] = true;
}

// ============================================================================
// Filter driving
// ============================================================================

template<class T>
void TcpFilter<T>::enqueueMeasurement(
  const std::string & topic_name, const Eigen::VectorXd & measurement,
  const Eigen::MatrixXd & measurement_covariance,
  const std::vector<bool> & update_vector, const double mahalanobis_thresh,
  const Time & time)
{
  MeasurementPtr meas = MeasurementPtr(new Measurement());

  meas->topic_name_ = topic_name;
  meas->measurement_ = measurement;
  meas->covariance_ = measurement_covariance;
  meas->update_vector_ = update_vector;
  meas->time_ = time;
  meas->mahalanobis_thresh_ = mahalanobis_thresh;

  // No control input in this pipeline.
  meas->latest_control_ = Eigen::VectorXd::Zero(0);
  meas->latest_control_time_ = Time(0.0);

  measurement_queue_.push(meas);
}

template<class T>
void TcpFilter<T>::integrateMeasurements(const Time & current_time)
{
  bool predict_to_current_time = predict_to_current_time_;

  if (!measurement_queue_.empty()) {
    const MeasurementPtr & first_measurement = measurement_queue_.top();

    if (smooth_lagged_data_ &&
      first_measurement->time_ < filter_.getLastMeasurementTime())
    {
      const Time first_measurement_time = first_measurement->time_;
      revertTo(Time(first_measurement_time.seconds() - 1e-9));
    }

    while (!measurement_queue_.empty()) {
      MeasurementPtr measurement = measurement_queue_.top();

      // Measurements are stored earliest-first, so once we hit one that's
      // still in the future, everything after it is too - stop here and
      // wait for a later iteration.
      if (current_time < measurement->time_) {
        break;
      }

      measurement_queue_.pop();

      // This calls predict() and, if appropriate, correct() internally.
      filter_.processMeasurement(*(measurement.get()));

      if (smooth_lagged_data_) {
        measurement_history_.push_back(measurement);

        if (measurement_queue_.empty() ||
          measurement_queue_.top()->time_ != filter_.getLastMeasurementTime())
        {
          saveFilterState(filter_);
        }
      }
    }
  } else if (filter_.getInitializedStatus()) {
    // No new measurements - if it's been a while, keep predicting forward
    // so the state estimate doesn't just freeze.
    Duration last_update_delta = current_time - filter_.getLastMeasurementTime();

    if (last_update_delta >= filter_.getSensorTimeout()) {
      predict_to_current_time = true;
    }
  }

  if (filter_.getInitializedStatus() && predict_to_current_time) {
    Duration last_update_delta = current_time - filter_.getLastMeasurementTime();

    filter_.validateDelta(last_update_delta);
    filter_.predict(current_time.seconds(), last_update_delta.seconds());

    filter_.setLastMeasurementTime(
      Time(filter_.getLastMeasurementTime().seconds() + last_update_delta.seconds()));
  }
}

template<class T>
void TcpFilter<T>::periodicUpdate()
{
  Time cur_time(nowSeconds());

  integrateMeasurements(cur_time);

  FilteredState state;
  if (getFilteredOdometry(state)) {
    if (!validateFilterOutput(state)) {
      std::cerr <<
        "Critical error: NaN/Inf detected in the filter output. This is "
        "usually caused by poorly-conditioned process/measurement "
        "covariances.\n";
    } else {
      last_published_stamp_ = Time(state.stamp_sec);

      if (print_output_) {
        printFilteredState(state);
      }
      if (send_filtered_output_) {
        sendFilteredOdom(state);
      }
    }
  }

  if (smooth_lagged_data_) {
    const Time cutoff(
      filter_.getLastMeasurementTime().seconds() - history_length_.seconds());
    clearExpiredHistory(cutoff);
  }
}

template<class T>
bool TcpFilter<T>::getFilteredOdometry(FilteredState & out)
{
  if (!filter_.getInitializedStatus()) {
    return false;
  }

  const Eigen::VectorXd & state = filter_.getState();
  const Eigen::MatrixXd & p = filter_.getEstimateErrorCovariance();

  out.x = state(StateMemberX);
  out.y = state(StateMemberY);
  out.z = state(StateMemberZ);
  out.roll = state(StateMemberRoll);
  out.pitch = state(StateMemberPitch);
  out.yaw = state(StateMemberYaw);

  rpyToQuaternion(out.roll, out.pitch, out.yaw, out.qx, out.qy, out.qz, out.qw);

  out.vx = state(StateMemberVx);
  out.vy = state(StateMemberVy);
  out.vz = state(StateMemberVz);
  out.vroll = state(StateMemberVroll);
  out.vpitch = state(StateMemberVpitch);
  out.vyaw = state(StateMemberVyaw);

  for (int i = 0; i < POSE_SIZE; ++i) {
    for (int j = 0; j < POSE_SIZE; ++j) {
      out.pose_covariance[POSE_SIZE * i + j] = p(i, j);
    }
  }

  for (int i = 0; i < TWIST_SIZE; ++i) {
    for (int j = 0; j < TWIST_SIZE; ++j) {
      out.twist_covariance[TWIST_SIZE * i + j] =
        p(i + POSITION_V_OFFSET, j + POSITION_V_OFFSET);
    }
  }

  out.stamp_sec = filter_.getLastMeasurementTime().seconds();
  out.frame_id = world_frame_id_;
  out.child_frame_id = base_link_output_frame_id_;

  return true;
}

template<class T>
bool TcpFilter<T>::validateFilterOutput(const FilteredState & state)
{
  bool ok = std::isfinite(state.x) && std::isfinite(state.y) &&
    std::isfinite(state.z) && std::isfinite(state.qx) &&
    std::isfinite(state.qy) && std::isfinite(state.qz) &&
    std::isfinite(state.qw) && std::isfinite(state.vx) &&
    std::isfinite(state.vy) && std::isfinite(state.vz) &&
    std::isfinite(state.vroll) && std::isfinite(state.vpitch) &&
    std::isfinite(state.vyaw);

  for (int i = 0; ok && i < 36; ++i) {
    if (!std::isfinite(state.pose_covariance[i]) ||
      !std::isfinite(state.twist_covariance[i]))
    {
      ok = false;
    }
  }

  return ok;
}

template<class T>
void TcpFilter<T>::printFilteredState(const FilteredState & state)
{


  // Convert stamp_sec (Unix epoch seconds) into a human-readable
  // "YYYY-MM-DD HH:MM:SS.mmm" string.
  time_t whole_sec = static_cast<time_t>(state.stamp_sec);
  int millis = static_cast<int>(
    (state.stamp_sec - static_cast<double>(whole_sec)) * 1000.0);

  struct tm tm_buf;
  localtime_r(&whole_sec, &tm_buf);   // use gmtime_r instead for UTC

  char time_str[32];
  strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);



  std::cout << std::fixed << std::setprecision(4)
             << "[t=" << time_str << "] "
             << "pos(" << state.x << ", " << state.y << ", " << state.z << ") "
             << "rpy(" << state.roll << ", " << state.pitch << ", " <<
    state.yaw << ") "
             << "vel(" << state.vx << ", " << state.vy << ", " << state.vz <<
    ") "
             << "ang_vel(" << state.vroll << ", " << state.vpitch << ", " <<
    state.vyaw << ")" << std::endl;
}

template<class T>
void TcpFilter<T>::sendFilteredOdom(const FilteredState & state)
{
  if (sockfd_ < 0) {
    return;
  }

  OdomData out;
  memset(&out, 0, sizeof(out));

  out.header.stamp.sec = static_cast<uint32_t>(state.stamp_sec);
  out.header.stamp.nanosec = static_cast<uint32_t>(
    (state.stamp_sec - static_cast<double>(out.header.stamp.sec)) * 1e9);
  std::strncpy(
    out.header.frame_id, state.frame_id.c_str(),
    sizeof(out.header.frame_id) - 1);
  std::strncpy(
    out.child_frame_id, state.child_frame_id.c_str(),
    sizeof(out.child_frame_id) - 1);

  out.pose.pose.position.x = state.x;
  out.pose.pose.position.y = state.y;
  out.pose.pose.position.z = state.z;
  out.pose.pose.orientation.x = state.qx;
  out.pose.pose.orientation.y = state.qy;
  out.pose.pose.orientation.z = state.qz;
  out.pose.pose.orientation.w = state.qw;

  for (int i = 0; i < 36; ++i) {
    out.pose.covariance[i] = state.pose_covariance[i];
  }

  out.twist.twist.linear.x = state.vx;
  out.twist.twist.linear.y = state.vy;
  out.twist.twist.linear.z = state.vz;
  out.twist.twist.angular.x = state.vroll;
  out.twist.twist.angular.y = state.vpitch;
  out.twist.twist.angular.z = state.vyaw;

  for (int i = 0; i < 36; ++i) {
    out.twist.covariance[i] = state.twist_covariance[i];
  }

  PacketHeader hdr;
  hdr.type = PACKET_FILTERED_ODOM;

  if (send(sockfd_, &hdr, sizeof(hdr), 0) != static_cast<ssize_t>(sizeof(hdr))) {
    std::cerr << "Warning: failed to send filtered-odom header.\n";
    return;
  }
  if (send(sockfd_, &out, sizeof(out), 0) != static_cast<ssize_t>(sizeof(out))) {
    std::cerr << "Warning: failed to send filtered-odom payload.\n";
  }
}

// ============================================================================
// Lagged-smoothing history
// ============================================================================

template<class T>
void TcpFilter<T>::saveFilterState(T & filter)
{
  FilterStatePtr new_state = FilterStatePtr(new FilterState());
  new_state->state_ = filter.getState();
  new_state->estimate_error_covariance_ = filter.getEstimateErrorCovariance();
  new_state->last_measurement_time_ = filter.getLastMeasurementTime().seconds();
  filter_state_history_.push_back(new_state);
}

template<class T>
bool TcpFilter<T>::revertTo(const Time & time)
{
  const double target = time.seconds();

  while (!filter_state_history_.empty() &&
    filter_state_history_.back()->last_measurement_time_ > target)
  {
    filter_state_history_.pop_back();
  }

  if (filter_state_history_.empty()) {
    return false;
  }

  FilterStatePtr restored = filter_state_history_.back();
  filter_.setState(restored->state_);
  filter_.setEstimateErrorCovariance(restored->estimate_error_covariance_);
  filter_.setLastMeasurementTime(Time(restored->last_measurement_time_));

  // Re-queue every measurement newer than the restored state so it gets
  // replayed forward on the next integrateMeasurements() pass.
  while (!measurement_history_.empty() &&
    measurement_history_.back()->time_ > time)
  {
    measurement_queue_.push(measurement_history_.back());
    measurement_history_.pop_back();
  }

  return true;
}

template<class T>
void TcpFilter<T>::clearExpiredHistory(const Time & cutoff_time)
{
  const double cutoff = cutoff_time.seconds();

  while (!filter_state_history_.empty() &&
    filter_state_history_.front()->last_measurement_time_ < cutoff)
  {
    filter_state_history_.pop_front();
  }

  while (!measurement_history_.empty() &&
    measurement_history_.front()->time_ < cutoff_time)
  {
    measurement_history_.pop_front();
  }
}

template<class T>
void TcpFilter<T>::clearMeasurementQueue()
{
  while (!measurement_queue_.empty()) {
    measurement_queue_.pop();
  }
}

// ============================================================================
// Math helpers (replace tf2::Quaternion / tf2::Matrix3x3)
// ============================================================================

template<class T>
void TcpFilter<T>::quaternionToRPY(
  double x, double y, double z, double w,
  double & roll, double & pitch, double & yaw)
{
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm > 1e-9) {
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  }

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::fabs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

template<class T>
void TcpFilter<T>::rpyToQuaternion(
  double roll, double pitch, double yaw,
  double & x, double & y, double & z, double & w)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
}

template<class T>
void TcpFilter<T>::rpyToRotationMatrix(
  double roll, double pitch, double yaw, double R[3][3])
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;

  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;

  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

// ============================================================================
// Main loop
// ============================================================================

template<class T>
void TcpFilter<T>::run()
{
  if (sockfd_ < 0) {
    std::cerr << "run() called before a successful connectToServer().\n";
    return;
  }

  const double period_sec = 1.0 / frequency_;
  double next_update = nowSeconds() + period_sec;

  while (true) {
    const double now = nowSeconds();
    double timeout_sec = next_update - now;
    if (timeout_sec < 0.0) {
      timeout_sec = 0.0;
    }

    struct pollfd pfd;
    pfd.fd = sockfd_;
    pfd.events = POLLIN;
    pfd.revents = 0;
std::cout << "Waiting in poll..." << std::endl;
    const int rv = poll(&pfd, 1, static_cast<int>(timeout_sec * 1000.0));
std::cout << "poll returned = " << rv
          << " revents = " << pfd.revents << std::endl;
    if (rv > 0 && (pfd.revents & POLLIN)) {
      if (!readOnePacket()) {
        std::cerr << "Connection closed by server.\n";
        break;
      }
    } else if (rv < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror("poll");
      break;
    }

    if (nowSeconds() >= next_update) {
      periodicUpdate();
      next_update += period_sec;

      // Don't try to burst-catch-up forever if we fell far behind.
      const double now2 = nowSeconds();
      if (next_update < now2) {
        next_update = now2 + period_sec;
      }
    }
  }
}

}  // namespace robot_localization

// ============================================================================
// Explicit instantiation
// ============================================================================
// Add another line here (with the matching #include) if you also have a
// standalone Ukf.
template class robot_localization::TcpFilter<robot_localization::Ekf>;

// ============================================================================
// main()
// ============================================================================

int main(int argc, char ** argv)
{
  std::string server_ip = "10.40.41.112";  // matches SERVER_IP in server.c
  int port = 10000;                        // matches PORT in server.c

  if (argc >= 2) {
    server_ip = argv[1];
  }
  if (argc >= 3) {
    port = std::atoi(argv[2]);
  }

  robot_localization::TcpFilter<robot_localization::Ekf> node(server_ip, port);

  // ---- Configuration ----------------------------------------------------
  // This replaces the YAML params the original ROS node used to load.

  node.two_d_mode_ = false;
  node.frequency_ = 20.0;                    // Hz - rate of periodicUpdate()
  node.predict_to_current_time_ = true;
  node.remove_gravitational_acceleration_ = true;
  node.gravitational_acceleration_ = 9.80665;
  node.smooth_lagged_data_ = false;
  node.print_output_ = true;

  // server.c only sends data - it never calls recv() - so sending fused
  // output back on the same socket would just pile up unread bytes on the
  // server side. Leave this off unless you extend server.c (or point this
  // program at a different listener) to actually consume
  // PACKET_FILTERED_ODOM packets.
  node.send_filtered_output_ = false;

  node.world_frame_id_ = "odom";
  node.base_link_output_frame_id_ = "base_link";

  // IMU: fuse orientation (roll/pitch/yaw), angular velocity, and linear
  // acceleration - the same three channels the original imuCallback() split
  // sensor_msgs/Imu into.
  node.imu_pose_config_ = {
    false, false, false, false, false, true,     // x, y, z, roll, pitch, yaw(is true b/c from yaml)
    false, false, false, false, false, false,  // vx..vyaw
    false, false, false                        // ax, ay, az
  };

    node.imu_twist_config_ = std::vector<bool>(robot_localization::STATE_SIZE, false);   // all false b/c from yaml 
  node.imu_accel_config_ = std::vector<bool>(robot_localization::STATE_SIZE, false);   // all false

 // vx, vy (indices 6,7) and vyaw (index 11) from its twist.
  node.odom_pose_config_ = std::vector<bool>(robot_localization::STATE_SIZE, false);   // all false

node.odom_twist_config_ = {
    false, false, false, false, false, false,
    true, true, false, false, false, true,     // vx, vy, -, -, -, vyaw
    false, false, false
  };

  if (!node.connectToServer()) {
    return 1;
  }

  node.run();

  return 0;
}

#ifndef EKF_MANAGER_HPP
#define EKF_MANAGER_HPP

#include <unordered_map>
#include <vector>
#include <queue>
#include"filter_time.hpp"

struct CallbackData
{
    std::vector<bool> update_vector;

    int update_sum;

    bool differential;

    bool relative;

    double rejection_threshold;
};
class EkfManager
{
public:

    EkfManager();




    std::unordered_map<std::string, Time> last_message_times_;

std::string base_link_frame_ = "base_link";

    // Called from TCP client
    
   void addImu(const ImuData& imu);


   void poseCallback(
    const PoseMeasurement& pose,
    const CallbackData& callback_data,
    const std::string& target_frame,
    bool imu_data);

    // Current EKF State
    // Eigen::VectorXd getState();
    Time last_set_pose_time_;

    struct PoseMeasurement
{
    Time timestamp;

    double qx;
    double qy;
    double qz;
    double qw;

    double covariance[36];
};


struct TwistMeasurement
{
    Time timestamp;

    double wx;
    double wy;
    double wz;

    double covariance[36];
};







private:


    CallbackData imu_pose_callback_data_;
    CallbackData imu_twist_callback_data_;
    CallbackData imu_accel_callback_data_;


   
};

#endif

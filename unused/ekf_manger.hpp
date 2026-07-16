#ifndef EKF_MANAGER_HPP
#define EKF_MANAGER_HPP

#include <queue>

#include "ekf.hpp"
#include "measurement.hpp"
#include "imu_data.hpp"
#include "odom_data.hpp"

class EkfManager
{
public:

    EkfManager();

    // Called from TCP client
    void addImu(const ImuData& imu);

    void addOdometry(const OdomData& odom);

    // Run EKF
    void process();

    // Current EKF State
    Eigen::VectorXd getState();

private:

    //--------------------------
    // IMU Processing
    //--------------------------

    void poseCallback(const ImuData& imu);

    void twistCallback(const ImuData& imu);

    void accelerationCallback(const ImuData& imu);

    //--------------------------
    // Odom Processing
    //--------------------------

    void poseCallback(const OdomData& odom);

    void twistCallback(const OdomData& odom);

    //--------------------------
    // Prepare Measurements
    //--------------------------

    bool preparePose(...);

    bool prepareTwist(...);

    bool prepareAcceleration(...);

    //--------------------------
    // Queue
    //--------------------------

    void enqueueMeasurement(
        const Measurement& measurement);

    void integrateMeasurements();

private:

    Ekf filter_;

    std::priority_queue<
        Measurement,
        std::vector<Measurement>,
        MeasurementCompare> measurement_queue_;
};

#endif

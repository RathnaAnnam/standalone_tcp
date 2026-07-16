#include <iostream>
#include <array>

//----------------------------------------------------
// Your IMU data structure
//----------------------------------------------------
struct ImuData
{
    double timestamp;

    // Orientation (Quaternion)
    double qx, qy, qz, qw;

    // Angular Velocity
    double wx, wy, wz;

    // Linear Acceleration
    double ax, ay, az;

    // Covariances
    std::array<double, 9> orientation_covariance;
    std::array<double, 9> angular_velocity_covariance;
    std::array<double, 9> linear_acceleration_covariance;
};

//----------------------------------------------------
// EKF Manager
//----------------------------------------------------
class EkfManager
{
public:

    void addImuData(const ImuData &imu)
    {
        std::cout << "Received IMU Data\n";

        //------------------------------------------------
        // 1. Check timestamp
        //------------------------------------------------
        if (imu.timestamp <= last_set_pose_time_)
        {
            std::cout << "Old IMU message. Ignore.\n";
            return;
        }

        //------------------------------------------------
        // 2. Orientation
        //------------------------------------------------
        if (use_orientation_)
        {
            if (imu.orientation_covariance[0] == -1)
            {
                std::cout << "Orientation not available.\n";
            }
            else
            {
                poseCallback(imu);
            }
        }

        //------------------------------------------------
        // 3. Angular Velocity
        //------------------------------------------------
        if (use_gyro_)
        {
            if (imu.angular_velocity_covariance[0] == -1)
            {
                std::cout << "Gyroscope not available.\n";
            }
            else
            {
                twistCallback(imu);
            }
        }

        //------------------------------------------------
        // 4. Linear Acceleration
        //------------------------------------------------
        if (use_acceleration_)
        {
            if (imu.linear_acceleration_covariance[0] == -1)
            {
                std::cout << "Acceleration not available.\n";
            }
            else
            {
                accelerationCallback(imu);
            }
        }
    }

private:

    //------------------------------------------
    // Same as poseCallback()
    //------------------------------------------
    void poseCallback(const ImuData &imu)
    {
        std::cout << "poseCallback()\n";

        // Next:
        // preparePose()
        // enqueueMeasurement()
    }

    //------------------------------------------
    // Same as twistCallback()
    //------------------------------------------
    void twistCallback(const ImuData &imu)
    {
        std::cout << "twistCallback()\n";

        // Next:
        // prepareTwist()
        // enqueueMeasurement()
    }

    //------------------------------------------
    // Same as accelerationCallback()
    //------------------------------------------
    void accelerationCallback(const ImuData &imu)
    {
        std::cout << "accelerationCallback()\n";

        // Next:
        // prepareAcceleration()
        // enqueueMeasurement()
    }

private:

    double last_set_pose_time_ = 0.0;

    bool use_orientation_ = true;
    bool use_gyro_ = true;
    bool use_acceleration_ = true;
};

//----------------------------------------------------
// Test
//----------------------------------------------------
int main()
{
    EkfManager ekf;

    ImuData imu;

    imu.timestamp = 1.0;

    imu.qx = 0;
    imu.qy = 0;
    imu.qz = 0;
    imu.qw = 1;

    imu.wx = 0.01;
    imu.wy = 0.02;
    imu.wz = 0.03;

    imu.ax = 0.1;
    imu.ay = 0.2;
    imu.az = 9.81;

    imu.orientation_covariance.fill(0.001);
    imu.angular_velocity_covariance.fill(0.001);
    imu.linear_acceleration_covariance.fill(0.001);

    ekf.addImuData(imu);

    return 0;
}

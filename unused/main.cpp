#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdint.h>
#include "ekf/header.hpp"
#include "ekf/ekf_manager.hpp"

#define SERVER_IP "10.40.41.112"   // Change to your server IP
#define PORT 10000

enum
{
    PACKET_IMU = 1,
    PACKET_ODOM = 2
};

struct PacketHeader
{
    uint32_t type;
};


bool recvAll(int sock, void *buffer, size_t size)
{
    char *ptr = static_cast<char *>(buffer);

    while (size > 0)
    {
        int bytes = recv(sock, ptr, size, 0);

        if (bytes <= 0)
        {
            return false;
        }

        ptr += bytes;
        size -= bytes;
    }

    return true;
}

int main()
{

      EkfManager ekf;
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    if (sock < 0)
    {
        perror("socket");
        return -1;
    }

    sockaddr_in server;

    memset(&server, 0, sizeof(server));

    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &server.sin_addr) <= 0)
    {
        perror("Invalid IP");
        return -1;
    }

    if (connect(sock, (sockaddr *)&server, sizeof(server)) < 0)
    {
        perror("connect");
        return -1;
    }

    std::cout << "======================================" << std::endl;
    std::cout << "Connected to Server" << std::endl;
    std::cout << "Server IP : " << SERVER_IP << std::endl;
    std::cout << "Port      : " << PORT << std::endl;
    std::cout << "======================================" << std::endl;

    while (true)
    {
        PacketHeader hdr;

        if (!recvAll(sock, &hdr, sizeof(hdr)))
        {
            std::cout << "Server disconnected" << std::endl;
            break;
        }

        if (hdr.type == PACKET_IMU)
        {
            ImuData imu;

            if (!recvAll(sock, &imu, sizeof(imu)))
                break;

                void addImu(const ImuData& imu);
//            std::cout << "\n========== IMU ==========\n";
                     
// std::cout << "Time : "
//           << imu.header.stamp.sec << "."
//           << imu.header.stamp.nanosec << std::endl;

// std::cout << "Frame ID : "
//           << imu.header.frame_id << std::endl;

// std::cout << "\nOrientation\n";
// std::cout << "x : " << imu.orientation.x << std::endl;
// std::cout << "y : " << imu.orientation.y << std::endl;
// std::cout << "z : " << imu.orientation.z << std::endl;
// std::cout << "w : " << imu.orientation.w << std::endl;


// std::cout << "\nOrientation Covariance\n";
// for(int i=0;i<9;i++)
//     std::cout << imu.orientation_covariance[i] << " ";

// std::cout << "\nAngular Velocity\n";
// std::cout << "x : " << imu.angular_velocity.x << std::endl;
// std::cout << "y : " << imu.angular_velocity.y << std::endl;
// std::cout << "z : " << imu.angular_velocity.z << std::endl;


// std::cout << "\nAngular Velocity Covariance\n";
// for(int i=0;i<9;i++)
//     std::cout << imu.angular_velocity_covariance[i]
//  << " ";

// std::cout << "\nLinear Acceleration\n";
// std::cout << "x : " << imu.linear_acceleration.x << std::endl;
// std::cout << "y : " << imu.linear_acceleration.y << std::endl;
// std::cout << "z : " << imu.linear_acceleration.z << std::endl;

// std::cout << "\nLinear Acceleration Covariance\n";
// for(int i=0;i<9;i++)
//     std::cout << imu.linear_acceleration_covariance[i]
//  << " ";

std::cout << std::endl;

            std::cout << "=========================\n";
        }
        else if (hdr.type == PACKET_ODOM)
        {
            OdomData odom;

            if (!recvAll(sock, &odom, sizeof(odom)))
                break;

             // ekf.addOdometry(odom);

            
//             std::cout << "\n========== ODOM ==========\n";

// std::cout << "Time : "
//           << odom.header.stamp.sec << "."
//           << odom.header.stamp.nanosec << std::endl;

// std::cout << "Frame ID : "
//           << odom.header.frame_id << std::endl;

// std::cout << "Child Frame : "
//           << odom.child_frame_id << std::endl;

// std::cout << "\nPosition\n";
// std::cout << "x : " << odom.pose.pose.position.x << std::endl;
// std::cout << "y : " << odom.pose.pose.position.y << std::endl;
// std::cout << "z : " << odom.pose.pose.position.z << std::endl;

// std::cout << "\nOrientation\n";
// std::cout << "x : " << odom.pose.pose.orientation.x << std::endl;
// std::cout << "y : " << odom.pose.pose.orientation.y << std::endl;
// std::cout << "z : " << odom.pose.pose.orientation.z << std::endl;
// std::cout << "w : " << odom.pose.pose.orientation.w << std::endl;

// std::cout << "\nLinear Velocity\n";
// std::cout << "x : " << odom.twist.twist.linear.x << std::endl;
// std::cout << "y : " << odom.twist.twist.linear.y << std::endl;
// std::cout << "z : " << odom.twist.twist.linear.z << std::endl;

// std::cout << "\nAngular Velocity\n";
// std::cout << "x : " << odom.twist.twist.angular.x << std::endl;
// std::cout << "y : " << odom.twist.twist.angular.y << std::endl;
// std::cout << "z : " << odom.twist.twist.angular.z << std::endl;

// std::cout << "\nPose Covariance\n";
// for(int i=0;i<36;i++)
//     std::cout << odom.pose.covariance[i] << " ";

// std::cout << std::endl;

// std::cout << "\nTwist Covariance\n";
// for(int i=0;i<36;i++)
//     std::cout << odom.twist.covariance[i] << " ";

// std::cout << std::endl;

//             std::cout << "==========================\n";
        }
        else
        {
            std::cout << "Unknown Packet Type: " << hdr.type << std::endl;
            break;
        }
        // ekf.process();
    }

    close(sock);

    return 0;
}

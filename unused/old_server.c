#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <time.h>
#include"header.h"
#define PORT 10000
#define SERVER_IP "10.40.41.112"    // <-- Change this to your server's IP

enum
{
    PACKET_IMU = 1,
    PACKET_ODOM = 2
};

typedef struct
{
    uint32_t type;
} PacketHeader;



// Formats ts as "YYYY-MM-DD HH:MM:SS.mmm" into buf (buf must be >= 24 bytes).
static void formatTimestamp(const struct timespec *ts, char *buf, size_t buf_len)
{
    struct tm tm_buf;
    localtime_r(&ts->tv_sec, &tm_buf);   // use gmtime_r instead for UTC

    char base[20];
    strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &tm_buf);

    snprintf(buf, buf_len, "%s.%03ld", base, ts->tv_nsec / 1000000);
}


int main()
{
    int serverfd = socket(AF_INET, SOCK_STREAM, 0);

    if (serverfd < 0)
    {
        perror("socket");
        return -1;
    }

    struct sockaddr_in addr;

    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    // Bind to a specific IP address
    if (inet_pton(AF_INET, SERVER_IP, &addr.sin_addr) <= 0)
    {
        perror("Invalid IP");
        return -1;
    }

    if (bind(serverfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return -1;
    }

    if (listen(serverfd, 1) < 0)
    {
        perror("listen");
        return -1;
    }

    printf("=========================================\n");
    printf("TCP Server Started\n");
    printf("Listening on %s:%d\n", SERVER_IP, PORT);
    printf("Waiting for client...\n");
    printf("=========================================\n");

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int client = accept(serverfd,
                        (struct sockaddr *)&client_addr,
                        &client_len);

    if (client < 0)
    {
        perror("accept");
        return -1;
    }

    printf("Client Connected\n");
    printf("Client IP   : %s\n", inet_ntoa(client_addr.sin_addr));
    printf("Client Port : %d\n", ntohs(client_addr.sin_port));

    while (1)
    {
        PacketHeader hdr;

        //---------------- IMU -----------------

        hdr.type = PACKET_IMU;

        ImuData imu;
        memset(&imu, 0, sizeof(imu));

//       imu.header.stamp.sec = 1769060351;
// imu.header.stamp.nanosec = 506578514;

struct timespec imu_ts;
clock_gettime(CLOCK_REALTIME, &imu_ts);
imu.header.stamp.sec = (uint32_t)imu_ts.tv_sec;
imu.header.stamp.nanosec = (uint32_t)imu_ts.tv_nsec;

strcpy(imu.header.frame_id, "imu_link");

imu.orientation.x = -0.011352097197632572;
imu.orientation.y = -0.010658752223891416;
imu.orientation.z = -0.8608581743207813;
imu.orientation.w = 0.5086066501682186;

for (int i = 0; i < 9; i++)
    imu.orientation_covariance[i] = 0.0;

imu.angular_velocity.x = 3.8582802834913476e-05;
imu.angular_velocity.y = 2.950449628552207e-05;
imu.angular_velocity.z = 0.0006502337066001595;

for (int i = 0; i < 9; i++)
    imu.angular_velocity_covariance[i] = 0.0;

imu.linear_acceleration.x = 0.00025000572204589847;
imu.linear_acceleration.y = 0.00011215209960937501;
imu.linear_acceleration.z = 0.004761791229248047;

for (int i = 0; i < 9; i++)
    imu.linear_acceleration_covariance[i] = 0.0;

        send(client, &hdr, sizeof(hdr), 0);
        send(client, &imu, sizeof(imu), 0);

        // printf("IMU packet sent\n");
        char imu_time_str[32];
        formatTimestamp(&imu_ts, imu_time_str, sizeof(imu_time_str));
        printf("[%s] IMU packet sent\n", imu_time_str);



        sleep(1);

        //---------------- ODOM -----------------

        hdr.type = PACKET_ODOM;

        OdomData odom;
        memset(&odom, 0, sizeof(odom));
 
//         odom.header.stamp.sec = 1769060335;
// odom.header.stamp.nanosec = 917362775;

  struct timespec odom_ts;
        clock_gettime(CLOCK_REALTIME, &odom_ts);
        odom.header.stamp.sec = (uint32_t)odom_ts.tv_sec;
        odom.header.stamp.nanosec = (uint32_t)odom_ts.tv_nsec;

strcpy(odom.header.frame_id, "odom");
strcpy(odom.child_frame_id, "base_link");

odom.pose.pose.position.x = 1.5861011304709287;
odom.pose.pose.position.y = 0.361115115324361;
odom.pose.pose.position.z = 0.0;

odom.pose.pose.orientation.x = 0.0;
odom.pose.pose.orientation.y = 0.0;
odom.pose.pose.orientation.z = 0.20362149445459807;
odom.pose.pose.orientation.w = 0.9790496856626205;

for (int i = 0; i < 36; i++)
    odom.pose.covariance[i] = 0.0;

odom.pose.covariance[0]  = 0.1;
odom.pose.covariance[7]  = 0.1;
odom.pose.covariance[14] = 0.1;
odom.pose.covariance[21] = 0.1;
odom.pose.covariance[28] = 0.1;
odom.pose.covariance[35] = 0.1;

odom.twist.twist.linear.x = 0.4740492669565562;
odom.twist.twist.linear.y = 0.006892477225540006;
odom.twist.twist.linear.z = 0.0;

odom.twist.twist.angular.x = 0.0;
odom.twist.twist.angular.y = 0.0;
odom.twist.twist.angular.z = -0.17453818297232665;

for (int i = 0; i < 36; i++)
    odom.twist.covariance[i] = 0.0;

odom.twist.covariance[0]  = 0.1;
odom.twist.covariance[7]  = 0.1;
odom.twist.covariance[14] = 0.1;
odom.twist.covariance[21] = 0.1;
odom.twist.covariance[28] = 0.1;
odom.twist.covariance[35] = 0.1;

        send(client, &hdr, sizeof(hdr), 0);
        send(client, &odom, sizeof(odom), 0);

        // printf("ODOM packet sent\n");
        char odom_time_str[32];
        formatTimestamp(&odom_ts, odom_time_str, sizeof(odom_time_str));
        printf("[%s] ODOM packet sent\n", odom_time_str);



        sleep(1);
    }

    close(client);
    close(serverfd);

    return 0;
}

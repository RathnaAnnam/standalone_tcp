#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <time.h>
#include"header.h"

#define DEST_PORT 10000
// IP address of the machine running the receiver (udp_filter / tcp_filter).
// <-- Change this to your receiver's IP.
#define DEST_IP "10.40.41.112"

enum
{
    PACKET_IMU = 1,
    PACKET_ODOM = 2
};

typedef struct
{
    uint32_t type;
} PacketHeader;

// UDP is message-oriented: one send = one datagram, and there is no
// "keep reading until you have N bytes" like a TCP stream. So header +
// payload must go out together in a single struct / single sendto() call,
// instead of the two separate send() calls the TCP version used.
typedef struct
{
    PacketHeader header;
    ImuData data;
} ImuPacket;

typedef struct
{
    PacketHeader header;
    OdomData data;
} OdomPacket;


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
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0)
    {
        perror("socket");
        return -1;
    }

    struct sockaddr_in dest_addr;

    memset(&dest_addr, 0, sizeof(dest_addr));

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);

    if (inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr) <= 0)
    {
        perror("Invalid IP");
        return -1;
    }

    // No bind()/listen()/accept() - UDP has no connection to accept.
    // We just send datagrams to DEST_IP:DEST_PORT below.

    printf("=========================================\n");
    printf("UDP Server Started\n");
    printf("Sending to %s:%d\n", DEST_IP, DEST_PORT);
    printf("=========================================\n");

    while (1)
    {
        //---------------- IMU -----------------

        ImuPacket imu_pkt;
        memset(&imu_pkt, 0, sizeof(imu_pkt));
        imu_pkt.header.type = PACKET_IMU;

struct timespec imu_ts;
clock_gettime(CLOCK_REALTIME, &imu_ts);
imu_pkt.data.header.stamp.sec = (uint32_t)imu_ts.tv_sec;
imu_pkt.data.header.stamp.nanosec = (uint32_t)imu_ts.tv_nsec;

strcpy(imu_pkt.data.header.frame_id, "imu_link");

imu_pkt.data.orientation.x = -0.011352097197632572;
imu_pkt.data.orientation.y = -0.010658752223891416;
imu_pkt.data.orientation.z = -0.8608581743207813;
imu_pkt.data.orientation.w = 0.5086066501682186;

for (int i = 0; i < 9; i++)
    imu_pkt.data.orientation_covariance[i] = 0.0;

imu_pkt.data.angular_velocity.x = 3.8582802834913476e-05;
imu_pkt.data.angular_velocity.y = 2.950449628552207e-05;
imu_pkt.data.angular_velocity.z = 0.0006502337066001595;

for (int i = 0; i < 9; i++)
    imu_pkt.data.angular_velocity_covariance[i] = 0.0;

imu_pkt.data.linear_acceleration.x = 0.00025000572204589847;
imu_pkt.data.linear_acceleration.y = 0.00011215209960937501;
imu_pkt.data.linear_acceleration.z = 0.004761791229248047;

for (int i = 0; i < 9; i++)
    imu_pkt.data.linear_acceleration_covariance[i] = 0.0;

        // One sendto() call = one datagram = header + payload together.
        ssize_t sent = sendto(sockfd, &imu_pkt, sizeof(imu_pkt), 0,
                               (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent != (ssize_t)sizeof(imu_pkt))
        {
            perror("sendto (imu)");
        }

        // printf("IMU packet sent\n");
        char imu_time_str[32];
        formatTimestamp(&imu_ts, imu_time_str, sizeof(imu_time_str));
        printf("[%s] IMU packet sent\n", imu_time_str);



        usleep(50000);           // 50,000 microseconds = 50 ms

        //---------------- ODOM -----------------

        OdomPacket odom_pkt;
        memset(&odom_pkt, 0, sizeof(odom_pkt));
        odom_pkt.header.type = PACKET_ODOM;

  struct timespec odom_ts;
        clock_gettime(CLOCK_REALTIME, &odom_ts);
        odom_pkt.data.header.stamp.sec = (uint32_t)odom_ts.tv_sec;
        odom_pkt.data.header.stamp.nanosec = (uint32_t)odom_ts.tv_nsec;

strcpy(odom_pkt.data.header.frame_id, "odom");
strcpy(odom_pkt.data.child_frame_id, "base_link");

odom_pkt.data.pose.pose.position.x = 1.5861011304709287;
odom_pkt.data.pose.pose.position.y = 0.361115115324361;
odom_pkt.data.pose.pose.position.z = 0.0;

odom_pkt.data.pose.pose.orientation.x = 0.0;
odom_pkt.data.pose.pose.orientation.y = 0.0;
odom_pkt.data.pose.pose.orientation.z = 0.20362149445459807;
odom_pkt.data.pose.pose.orientation.w = 0.9790496856626205;

for (int i = 0; i < 36; i++)
    odom_pkt.data.pose.covariance[i] = 0.0;

odom_pkt.data.pose.covariance[0]  = 0.1;
odom_pkt.data.pose.covariance[7]  = 0.1;
odom_pkt.data.pose.covariance[14] = 0.1;
odom_pkt.data.pose.covariance[21] = 0.1;
odom_pkt.data.pose.covariance[28] = 0.1;
odom_pkt.data.pose.covariance[35] = 0.1;

odom_pkt.data.twist.twist.linear.x = 0.4740492669565562;
odom_pkt.data.twist.twist.linear.y = 0.006892477225540006;
odom_pkt.data.twist.twist.linear.z = 0.0;

odom_pkt.data.twist.twist.angular.x = 0.0;
odom_pkt.data.twist.twist.angular.y = 0.0;
odom_pkt.data.twist.twist.angular.z = -0.17453818297232665;

for (int i = 0; i < 36; i++)
    odom_pkt.data.twist.covariance[i] = 0.0;

odom_pkt.data.twist.covariance[0]  = 0.1;
odom_pkt.data.twist.covariance[7]  = 0.1;
odom_pkt.data.twist.covariance[14] = 0.1;
odom_pkt.data.twist.covariance[21] = 0.1;
odom_pkt.data.twist.covariance[28] = 0.1;
odom_pkt.data.twist.covariance[35] = 0.1;

        sent = sendto(sockfd, &odom_pkt, sizeof(odom_pkt), 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent != (ssize_t)sizeof(odom_pkt))
        {
            perror("sendto (odom)");
        }

        // printf("ODOM packet sent\n");
        char odom_time_str[32];
        formatTimestamp(&odom_ts, odom_time_str, sizeof(odom_time_str));
        printf("[%s] ODOM packet sent\n", odom_time_str);



        sleep(1);
    }

    close(sockfd);

    return 0;
}

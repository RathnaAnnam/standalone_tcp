#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

typedef struct
{
    uint32_t sec;
    uint32_t nanosec;
} PacketTime;

typedef struct
{
    PacketTime stamp;
    char frame_id[64];
} Header;

typedef struct
{
    double x;
    double y;
    double z;
} Vector3;

typedef struct
{
    double x;
    double y;
    double z;
    double w;
} Quaternion;

typedef struct
{
    Vector3 position;
    Quaternion orientation;
} Pose;

typedef struct
{
    Pose pose;
    double covariance[36];
} PoseWithCovariance;

typedef struct
{
    Vector3 linear;
    Vector3 angular;
} Twist;

typedef struct
{
    Twist twist;
    double covariance[36];
} TwistWithCovariance;

typedef struct
{
    Header header;

    Quaternion orientation;
    double orientation_covariance[9];

    Vector3 angular_velocity;
    double angular_velocity_covariance[9];

    Vector3 linear_acceleration;
    double linear_acceleration_covariance[9];

} ImuData;

typedef struct
{
    Header header;

    char child_frame_id[64];

    PoseWithCovariance pose;

    TwistWithCovariance twist;

} OdomData;

#endif

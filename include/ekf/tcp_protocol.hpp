#ifndef TCP_PROTOCOL_HPP_
#define TCP_PROTOCOL_HPP_

#include <stdint.h>

// Mirrors the packet header used by server.c:
//   typedef struct { uint32_t type; } PacketHeader;
typedef struct
{
    uint32_t type;
} PacketHeader;

enum PacketType
{
    PACKET_IMU = 1,
    PACKET_ODOM = 2,
    // Not sent by the original server.c - used only if this program is
    // configured to write its fused output back over the same socket.
    PACKET_FILTERED_ODOM = 3
};



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

typedef struct
{
    PacketHeader header;
    OdomData data;  // filtered output reuses the OdomData layout
} FilteredOdomPacket;


#endif  // TCP_PROTOCOL_HPP_

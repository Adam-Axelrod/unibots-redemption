// Oradar MS200 packet parser. Pure C, tested on a laptop.
//
// A packet is 47 bytes: 0x54, 0x2C, speed(2), start angle(2), 12 x [distance(2),
// intensity(1)], end angle(2), timestamp(2), CRC8. Multi-byte values are little
// endian and angles are in 0.01 degree, clockwise, 0 at the lidar's arrow.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define MS200_POINTS 12
#define MS200_PACKET_LEN (11 + 3 * MS200_POINTS)
#define MS200_BINS 360

typedef struct {
    uint8_t buf[MS200_PACKET_LEN];
    int len;
} ms200_parser_t;

uint8_t ms200_crc8(const uint8_t *data, int len);

// Feed one byte. Returns true when buf holds a complete packet with a good CRC.
bool ms200_feed(ms200_parser_t *p, uint8_t byte);

// Store a packet's distances (mm) by whole degree, indexed as in LaserScan: index 0
// is -180 degrees (behind the robot), 180 is ahead, angles anticlockwise.
void ms200_store(const uint8_t *packet, uint16_t distances_mm[MS200_BINS]);

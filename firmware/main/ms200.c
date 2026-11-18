#include "ms200.h"

// CRC-8, polynomial 0x4D: checked against the table in Oradar's driver.
uint8_t ms200_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool ms200_feed(ms200_parser_t *p, uint8_t byte)
{
    if (p->len == 0 && byte != 0x54) return false;
    if (p->len == 1 && byte != 0x2C) {
        p->len = (byte == 0x54) ? 1 : 0;
        return false;
    }
    p->buf[p->len++] = byte;
    if (p->len < MS200_PACKET_LEN) return false;

    p->len = 0;
    return ms200_crc8(p->buf, MS200_PACKET_LEN - 1) == p->buf[MS200_PACKET_LEN - 1];
}

static inline uint16_t le16(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }

void ms200_store(const uint8_t *packet, uint16_t distances_mm[MS200_BINS])
{
    uint32_t start = le16(&packet[4]);
    uint32_t end = le16(&packet[6 + 3 * MS200_POINTS]);
    uint32_t span = (end >= start) ? end - start : 36000 + end - start;

    for (int i = 0; i < MS200_POINTS; i++) {
        uint32_t centideg = start + span * (uint32_t)i / (MS200_POINTS - 1);
        uint32_t lidar_deg = (centideg / 100) % MS200_BINS;
        uint32_t index = (540 - lidar_deg) % MS200_BINS;   // clockwise to LaserScan
        distances_mm[index] = le16(&packet[6 + 3 * i]);
    }
}

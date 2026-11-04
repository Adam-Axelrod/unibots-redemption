#pragma once
#include <stdbool.h>

bool imu_init(void);                                   // false if the chip did not answer
bool imu_read(float accel_mps2[3], float gyro_rads[3]);

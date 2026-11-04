#pragma once
#include "ms200.h"

void lidar_init(void);
void lidar_copy_ranges(float ranges_m[MS200_BINS]);

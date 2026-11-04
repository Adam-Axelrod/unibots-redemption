#pragma once
#include <stdbool.h>
#include "mecanum.h"

void motors_init(void);
void motors_command(const twist_t *cmd);      // Also feeds the command watchdog
void motors_measured(twist_t *out);           // Body velocity from the encoders
bool motors_faulted(void);

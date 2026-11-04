#include <math.h>
#include "mecanum.h"

void mecanum_inverse(const twist_t *cmd, float k, float max_speed, float wheels[4])
{
    float turn = cmd->wz * k;
    wheels[0] = cmd->vx - cmd->vy - turn;
    wheels[1] = cmd->vx + cmd->vy - turn;
    wheels[2] = cmd->vx + cmd->vy + turn;
    wheels[3] = cmd->vx - cmd->vy + turn;

    float largest = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (fabsf(wheels[i]) > largest) largest = fabsf(wheels[i]);
    }
    if (largest > max_speed) {
        for (int i = 0; i < 4; i++) wheels[i] *= max_speed / largest;
    }
}

void mecanum_forward(const float wheels[4], float k, twist_t *out)
{
    out->vx = (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4.0f;
    out->vy = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4.0f;
    out->wz = (-wheels[0] - wheels[1] + wheels[2] + wheels[3]) / (4.0f * k);
}

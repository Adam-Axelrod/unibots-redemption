// Mecanum wheel maths. Pure C with no ESP-IDF, so it is tested on a laptop.
//
// Wheel order: left front, left rear, right front, right rear. Assumes the rollers
// on the tops of the wheels form an X seen from above. k is (track + base) / 2.
#pragma once

typedef struct {
    float vx;   // m/s, forward
    float vy;   // m/s, left
    float wz;   // rad/s, anticlockwise from above
} twist_t;

// Body velocity to wheel speeds. Scales all four down together if any would exceed
// max_speed, so the robot still moves in the commanded direction.
void mecanum_inverse(const twist_t *cmd, float k, float max_speed, float wheels[4]);

// Wheel speeds to body velocity.
void mecanum_forward(const float wheels[4], float k, twist_t *out);

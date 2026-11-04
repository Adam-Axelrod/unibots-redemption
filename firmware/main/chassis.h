// Everything about the physical robot. When the chassis changes, this is the only
// firmware file to edit: firmware/README.md#when-the-chassis-changes
#pragma once

// ---- Measure in millimetres. -------------------------------------------------
#define WHEEL_DIAMETER_MM  52     // Across the rollers. Sets speed and distance
#define TRACK_WIDTH_MM     135    // Left wheel centre to right wheel centre: 110 gap + 25 wheel
#define WHEEL_BASE_MM      95     // Front axle to rear axle. Sets turning

// ---- Motors. Only changes if the motors or gearboxes do. ----------------------
// Encoder pulses for one turn of the wheel. Yahboom's figure is 1040.
#define ENCODER_PULSES_PER_REV 1040

// ---- Wiring. Fix a direction here if a motor is rewired. ----------------------
// Order: left front (port 1), left rear (2), right front (3), right rear (4).
#define MOTOR_PWM_INVERT   {false, false, true, true}     // Wheel turns the wrong way
#define MOTOR_ENC_INVERT   {false, false, true, true}     // Speed reads backwards

// Catch typos, such as metres or centimetres, when building rather than on the floor.
_Static_assert(WHEEL_DIAMETER_MM >= 30 && WHEEL_DIAMETER_MM <= 150, "WHEEL_DIAMETER_MM looks wrong: use millimetres");
_Static_assert(TRACK_WIDTH_MM >= 50 && TRACK_WIDTH_MM <= 300, "TRACK_WIDTH_MM looks wrong: use millimetres");
_Static_assert(WHEEL_BASE_MM >= 50 && WHEEL_BASE_MM <= 300, "WHEEL_BASE_MM looks wrong: use millimetres");

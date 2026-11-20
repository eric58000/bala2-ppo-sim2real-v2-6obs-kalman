# BALA2 Fire - Testing the NEW PPO Model with Raw Outputs

This firmware controls the wheels for autonomous balancing. It adds NO corrections to the PPO agent: neither PID nor differential correction. The raw actions are only clamped to [-1, 1] (the physical limit of the motor driver). Safety protections may stop the motors but never modify the predictions.

## Observations

`[pitch, pitch_rate, wheel_vel_L, wheel_vel_R, wheel_pos_L, wheel_pos_R]` in rad, rad/s, rad/s, rad/s, rad, rad, respectively. `pitch` and `pitch_rate` come from the embedded two-state Kalman filter. Positions are measured in radians from `ClearEncoder()` when balancing is armed. Velocity is calculated from the difference between encoder counts over 25 ms (5 cycles of 5 ms); resolution: 414 ticks/revolution. Encoder directions +1/+1 and motor directions -1/-1 were verified through testing.

## Usage

Board: `m5stack:esp32:m5stack_fire`; library: `M5Unified`. SPIFFS is available for CSV logging (not required for control). At startup, keep the device stationary for approximately two seconds to estimate the gyro bias. Existing M5Unified IMU calibration stored in NVS is required. Button A engages balancing only when the robot is close to upright; button B stops the motors and saves the CSV. Serial at 115200 baud: `X` to stop, `D` to dump the CSV while stopped, `E` to erase it, and `H` for help. Saving overwrites the previous log.

The logging buffer is allocated in PSRAM (120 s at 10 Hz), or internal RAM (25 s), or disabled if allocation fails. No large static allocation is used.

## Safety

Test with a fall-prevention device and sufficient clear space. Keep the first tests very short. The limits of 45° (Kalman), 60° (valid accelerometer), 6 rad of wheel-position difference, and 15 rad of mean wheel position are protective stop thresholds, not feedback controllers.

The firmware cannot be considered validated without compiling and testing it on your hardware. Do not use it if the action direction conventions have changed in the model.

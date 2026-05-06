# STEM Robot Satellite Simulator

This repository contains the Arduino code for running the STEM robot satellite simulator.

## How to Calibrate:
1. Set the robot to move at full speed (you can do this by sending an impulse with high magnitude)
2. Measure how far it travels in a known time, or time over a known distance
3. Calculate: scaling_factor = 255 / actual_speed_in_m_per_s
4. Enter this value in the web interface
- For example, if your robot goes 0.5 m/s at PWM 255, set scaling factor to 510. Then, to achieve 1 m/s, the code will use PWM = 1 * 510 = 510, but it will be capped at 255 (full speed).
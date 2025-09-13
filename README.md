# Self‑Balancing Robot (STM32F407)

<p align="center">
  <img src="Balanced.gif" alt="Self-balancing robot demo" width="640" />
</p>

A compact two‑wheel, self‑balancing robot built around an STM32F407 MCU, an MPU‑6050 IMU for angle sensing, and an L298N dual H‑bridge for motor drive. The firmware runs a 200 Hz control loop with a complementary filter for attitude estimation and a PID controller to keep the robot upright.

## Hardware

- MCU: STM32F407 (F4 series)
- IMU: InvenSense MPU‑6050 (accelerometer + gyroscope, I²C @ 400 kHz)
- Motor driver: L298N dual H‑bridge (PWM + DIR)
- Motors: 3–6 V DC gear motors (x2)
- Power: 12 V transformer for motor driver

## Key Features

- 200 Hz control loop (TIM6) with DMA I²C burst reads from the MPU‑6050
- Complementary filter (gyro + accel) for robust pitch estimation
- PID balance controller with integral windup clamp and safety angle cutoff
- PWM motor output (TIM1 CH1) and GPIO DIR lines for forward/reverse
- High‑speed UART logging (921600 baud) for tuning/telemetry

## Control and Estimation

- Attitude: complementary filter
  - pitch_acc = atan2(-ax, az)
  - pitch = α · (prev + gyro_y · dt) + (1−α) · pitch_acc (α≈0.95, dt=0.005 s)
- PID (angle hold)
  - error = pitch_setpoint − pitch
  - u = Kp·error + Ki·∫error·dt + Kd·(−gyro_y)
  - Integral clamp and deadband applied; output mapped to PWM duty and DIR
- Safety: motors are cut off if |pitch| exceeds a configured threshold

## Firmware Overview

- Timer‑driven sampling: TIM6 @ 200 Hz triggers MPU‑6050 burst read via I²C DMA
- Processing: on RX complete, the main loop converts raw data, updates the filter, runs PID, and updates PWM/DIR
- Output: TIM1 CH1 PWM for duty, GPIO pins for direction; UART prints 1 Hz status

## Tuning Tips

1. Start with Kd = 0. Increase Kp until the robot begins to balance but oscillates.
2. Add Kd to damp oscillations.
3. Add small Ki to remove steady‑state tilt; clamp integral to prevent windup.
4. Re‑check α (complementary filter) and sensor offsets if drift/noise appears.

## Build and Flash

- Open the folder in STM32CubeIDE
- Build the project (Debug or Release)
- Flash to the target with ST‑Link

## Notes

- Double‑check motor voltage/current vs. L298N limits; add proper power decoupling.
- Ensure IMU is mounted rigidly with the expected axes orientation.
- Share ground between logic, driver, and power supply.

## Credits

- LANSARI Fedi <lansarifedi7@gmail.com>
- GHARBI Yassine <gharbiyasine040@gmail.com>

## License

GPL‑3.0‑or‑later. See `LICENSE` for details.

---

Feel free to open issues or PRs for improvements, documentation, or features.

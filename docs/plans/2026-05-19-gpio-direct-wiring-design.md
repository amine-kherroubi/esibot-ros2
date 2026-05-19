# Design — GPIO Direct Wiring (v2 wiring diagram)

Date: 2026-05-19

## Context

New wiring removes ESP32 from motor and servo control loops:
- MG996R servo: Pi GPIO12 (hardware PWM) → 1kΩ → servo signal
- L298N motors: Pi GPIO5/6/13/19 → IN1/IN2/IN3/IN4 directly
- ESP32: camera + BAT telemetry only (UART read-only from Pi side)

## Changes

### esibot_driver.py
- Add motor GPIO pins: IN1=GPIO5, IN2=GPIO6, IN3=GPIO13, IN4=GPIO19
- `_setup_motor_gpio()`: configure 4 OUTPUT pins
- `_set_motor(v_left, v_right)`: binary control (full speed or stop)
- `_cmd_vel_callback`: remove UART CMD send, call `_set_motor()`
- `_send_stop`: call `_set_motor(0,0)`
- UART stays open for BAT: read-only

### radar_node.py
- Remove: pyserial, UART params, `_send_servo_angle`, UART init/cleanup
- Add: pigpio, `_set_servo_angle()` via `pi.set_servo_pulsewidth(12, us)`
- Pulse formula: `1500 + degrees(angle) * (500/90)` µs → 1000µs=-90°, 2000µs=+90°
- HC-SR04 GPIO23/24 unchanged

## Motor control logic (binary)
- v > 0.05 → forward (IN_fwd=HIGH, IN_rev=LOW)
- v < -0.05 → backward (IN_fwd=LOW, IN_rev=HIGH)
- |v| ≤ 0.05 → stop (both LOW)

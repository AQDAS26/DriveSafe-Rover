# DriveSafe Rover 2026 – Final Circuit Connections

See GPIO mapping and wiring below.

## GPIO Mapping
- GPIO18 -> L298N #1 ENA
- GPIO19 -> L298N #1 IN1
- GPIO23 -> L298N #1 IN2
- GPIO5 -> L298N #2 ENB
- GPIO4 -> L298N #2 IN3
- GPIO2 -> L298N #2 IN4
- GPIO13 -> Servo Signal
- GPIO26 -> Shared HC-SR04 Trigger
- GPIO27 -> Radar Echo
- GPIO14 -> Pothole Echo
- GPIO16 -> GPS RX
- GPIO17 -> GPS TX
- GPIO21 -> Headlights
- GPIO22 -> Tail Lights
- GPIO32 -> Hazard Lights

## Power
3S Battery -> XT30 -> 10A Fuse -> Rocker Switch -> 12V Rail.
12V Rail -> L298N #1, L298N #2, LM2596 Input.
LM2596 +5V Output -> ESP32 VIN, ESP32-CAM 5V, GPS, Servo, Both HC-SR04.
All grounds common.

## Motors
L298N #1: Front Left + Rear Left.
L298N #2: Front Right + Rear Right.

## ESP32-CAM
Only 5V and GND connected.
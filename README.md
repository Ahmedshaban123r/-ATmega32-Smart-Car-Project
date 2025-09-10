# ATmega32 Autonomous Car System

## 📌 Project Overview
This project implements a fully autonomous smart car using the **ATmega32 microcontroller** with three core capabilities:

- **Lane Tracking** – Follows black lanes with white borders using IR sensors  
- **Obstacle Avoidance** – Detects and navigates around obstacles using ultrasonic sensors  
- **Adaptive Speed Control** – Dynamically adjusts speed based on road conditions and obstacle proximity  

The system features a **state-machine architecture** with non-blocking operations and professional-grade embedded software design.

---

## 🛠️ Hardware Components
- **ATmega32 Microcontroller (16MHz)**
- **3x IR Sensors** (Left, Center, Right) – Line detection
- **3x Ultrasonic Sensors** (Front, Left, Right) – Obstacle detection
- **L293D Motor Driver** – H-bridge motor control
- **4x DC Motors** (2 left, 2 right) – Movement system
- **Li-ion Battery Pack** – Power supply

---

## 🔌 Pin Configuration

| Peripheral       | ATmega32 Pins              | Function                  |
|------------------|----------------------------|---------------------------|
| Motor PWM        | OC1A (PD5), OC1B (PD4)     | 10-bit PWM speed control |
| Motor Direction  | PD2, PD3, PD6, PD7         | IN1–IN4 for L293D        |
| IR Sensors       | PC0 (Left), PC1 (Center), PC2 (Right) | Digital input with pull-ups |
| Ultrasonic Trig  | PB0 (Front), PB2 (Left), PB4 (Right) | Trigger pulses |
| Ultrasonic Echo  | PB1 (Front), PB3 (Left), PB5 (Right) | Echo input |

---

## ⚙️ Software Architecture

### 📁 Project Structure
SmartCar_ATmega32/
├── Main.c # Primary state machine and control logic
├── Motor.h / Motor.c # Motor control (PWM + direction)
├── IR.h / IR.c # IR sensor interface
├── Ultrasonic.h / Ultrasonic.c # Ultrasonic driver
├── TIMER_interface.h # Timer configuration API
├── TIMER_private.h # Timer register definitions
├── TIMER_config.h # Timer default settings
└── TIMER_program.c # Timer implementation

---

## 🧠 Core Features
- **10-bit PWM Motor Control** – Smooth speed regulation (0–1023 resolution)  
- **Non-blocking Architecture** – Millis-based timing for responsive operation  
- **State Machine Design** – Three main states with sub-state management  
- **Proportional Line Following** – Error-based steering correction  
- **Intelligent Obstacle Avoidance** – Multi-stage avoidance maneuvers  
- **Systematic Search Pattern** – Zigzag recovery for lost line scenarios  

---

## 🎯 Control Logic

### 🔄 State Machine Overview

    typedef enum {
    STATE_PROCESS_IR,    // Normal lane following
    STATE_STOP_AVOID,    // Obstacle avoidance
    STATE_SEARCH         // Line search recovery
    } CarState_t;
    #define SPEED_NORMAL 180   // 0-255 scale (converted to 10-bit PWM)
    #define SPEED_SLOW   100   // Slow speed for obstacle proximity
    #define SPEED_AVOID  150   // Avoidance maneuver speed
    #define SPEED_SEARCH 120   // Search pattern speed


🚦 State Transitions

STATE_PROCESS_IR → Normal operation, line detected

STATE_STOP_AVOID → Obstacle detected < 10cm

STATE_SEARCH → All IR sensors white (line lost)
---
⏱️ Timeout protection on all states (5-second maximum).

⚡ Real-Time Operation
⏰ Timing Configuration

System clock: Timer0 CTC mode @ 1kHz (1ms ticks)

Ultrasonic sampling: Every 100ms

State machine updates: Every 50ms

PWM frequency: ~2kHz (16MHz/8/1024)
---
🎮 Control Behaviors

Lane Following – Proportional control based on IR sensor patterns

Obstacle Avoidance – 8-stage maneuver with backup and turn decisions

Line Recovery – Systematic zigzag search with backup sequence
---
🚀 Key Features
✅ Implemented

Smooth motor control with 10-bit PWM resolution

Non-blocking sensor reading with timeout protection

Advanced state management with safety timeouts

Proportional steering control for precise lane tracking

Multi-stage obstacle avoidance with ultrasonic decision making

Configurable parameters for easy tuning
---
🔮 Future Enhancements

PID controller for smoother line following

Bluetooth remote control override

UART telemetry for debugging and monitoring

Battery voltage monitoring and low-power modes

Machine learning for improved decision making
---
📋 Building and Deployment
🔧 Compilation
avr-gcc -mmcu=atmega32 -Os -DF_CPU=16000000UL *.c -o smartcar.elf
avr-objcopy -O ihex smartcar.elf smartcar.hex
---
⚡ Flashing
avrdude -c usbasp -p m32 -U flash:w:smartcar.hex
---
🧪 Testing

Verify motor directions and IR sensor responses

Calibrate ultrasonic distances for your environment

Test on actual track with various obstacle scenarios

Tune parameters (speeds, timeouts, thresholds) as needed
---
🎓 Educational Value

This project demonstrates mastery of:

Embedded C programming for AVR microcontrollers

Real-time system design with state machines

Hardware interfacing (sensors, motors, timers)

PWM-based motor control techniques

Sensor integration and data processing

System architecture and modular design



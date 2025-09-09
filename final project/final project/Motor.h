#ifndef _MOTOR_H
#define _MOTOR_H

#include <avr/io.h>
#include <stdint.h>
#include "TIMER_interface.h"

/* MOTOR PINS (physical MCU pins used for IN1..IN4 and OC1A/OC1B) */
#define IN1_PIN PD2
#define IN2_PIN PD3
#define IN3_PIN PD6
#define IN4_PIN PD7

/* PWM channels map:
   - Left motor PWM -> OCR1A (OC1A) -> TIMER_ID_1, TIMER_CH_A
   - Right motor PWM -> OCR1B (OC1B) -> TIMER_ID_1, TIMER_CH_B
*/

/* MOTOR DIRECTIONS ENUM */
typedef enum {
    FORWARD  = 0x05,   // 0101 -> IN1=1, IN2=0, IN3=1, IN4=0
    BACKWARD = 0x0A,   // 1010 -> IN1=0, IN2=1, IN3=0, IN4=1
    RIGHT    = 0x09,   // 1001 -> IN1=1, IN2=0, IN3=0, IN4=1
    LEFT     = 0x06,   // 0110 -> IN1=0, IN2=1, IN3=1, IN4=0
    STOP     = 0x00    // 0000
} Motor_Directions;

/* API */
void motor_init(void);
/* speeds: 0..255 (8-bit) — because TIMER_setDutyRaw uses 8-bit mapping in your MCAL */
void motor_set_speed(uint8_t left_spd, uint8_t right_spd);
/* direction + speeds */
void motor_control(Motor_Directions dir, uint8_t left_spd, uint8_t right_spd);

#endif /* _MOTOR_H */

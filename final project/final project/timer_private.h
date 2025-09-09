/*
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TIMER_private.h >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Author : Ahmed Shaban
 * Layer  : MCAL
 * SWC    : TIMER (Unified for Timer0, Timer1, Timer2)
 */
#include <avr/io.h>
#ifndef TIMER_PRIVATE_H_
#define TIMER_PRIVATE_H_

/* ===================== Timer0 Registers ===================== */
#define TCCR0   (*(volatile uint8_t*)0x53)
#define TCNT0   (*(volatile uint8_t*)0x52)
#define OCR0    (*(volatile uint8_t*)0x5C)

/* ===================== Timer1 Registers (16-bit) ============ */
#define TCCR1A  (*(volatile uint8_t*)0x4F)
#define TCCR1B  (*(volatile uint8_t*)0x4E)
#define TCNT1   (*(volatile uint16_t*)0x4C)
#define OCR1A   (*(volatile uint16_t*)0x4A)
#define OCR1B   (*(volatile uint16_t*)0x48)
#define ICR1    (*(volatile uint16_t*)0x46)

/* ===================== Timer2 Registers ===================== */
#define TCCR2   (*(volatile uint8_t*)0x45)
#define TCNT2   (*(volatile uint8_t*)0x44)
#define OCR2    (*(volatile uint8_t*)0x43)

/* ===================== Interrupt Registers ================== */
#define TIMSK   (*(volatile uint8_t*)0x59)
#define TIFR    (*(volatile uint8_t*)0x58)

/* ===================== Bit Macros =========================== */
/* Timer0 (TCCR0) */
#define CS00   0
#define CS01   1
#define CS02   2
#define WGM01  3
#define COM00  4
#define COM01  5
#define WGM00  6
#define FOC0   7

/* Timer1 (TCCR1A) */
#define WGM10  0
#define WGM11  1
#define FOC1B  2  // Force Output Compare B (non-PWM)
#define FOC1A  3  // Force Output Compare A (non-PWM)
#define COM1B0 4
#define COM1B1 5
#define COM1A0 6
#define COM1A1 7

/* Timer1 (TCCR1B) */
#define CS10   0
#define CS11   1
#define CS12   2
#define WGM12  3
#define WGM13  4
#define ICES1  6
#define ICNC1  7

/* Timer2 (TCCR2) */
#define CS20   0
#define CS21   1
#define CS22   2
#define WGM21  3
#define COM20  4
#define COM21  5
#define WGM20  6
#define FOC2   7

/* TIMSK (Interrupt Enable) */
#define TOIE0   0
#define OCIE0   1
#define TOIE1   2
#define OCIE1B  3
#define OCIE1A  4
#define TICIE1  5
#define TOIE2   6
#define OCIE2   7

/* TIFR (Interrupt Flags) */
#define TOV0    0
#define OCF0    1
#define TOV1    2
#define OCF1B   3
#define OCF1A   4
#define ICF1    5
#define TOV2    6
#define OCF2    7

/* OC pins */
#define OC0_DDR  DDRB
#define OC0_PIN  PB3

#define OC1A_DDR DDRD
#define OC1A_PIN PD5
#define OC1B_DDR DDRD
#define OC1B_PIN PD4

#define OC2_DDR  DDRD
#define OC2_PIN  PD7

#endif /* TIMER_PRIVATE_H_ */

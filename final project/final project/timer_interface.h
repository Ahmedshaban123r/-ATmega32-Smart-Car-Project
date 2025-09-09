/*
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<    TIMER_interface.h    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *
 *  Author : Ahmed Shaban
 *  Layer  : MCAL
 *  SWC    : TIMER
 *
 */

#ifndef TIMER_INTERFACE_H_
#define TIMER_INTERFACE_H_

#include <stdint.h>

/* ===================== Timer Selection ===================== */
typedef enum {
    TIMER_ID_0 = 0,
    TIMER_ID_1,
    TIMER_ID_2
} TIMER_ID_t;

/* ===================== Channels (where applicable) ===================== */
typedef enum {
    TIMER_CH_A = 0,   // Timer0: OC0, Timer1: OC1A, Timer2: OC2
    TIMER_CH_B        // Only valid for Timer1 (OC1B)
} TIMER_Channel_t;

/* ===================== Operating Modes ===================== */
typedef enum {
    TIMER_MODE_NORMAL = 0,   // Overflow mode
    TIMER_MODE_PHASE_PWM,    // Phase Correct PWM
    TIMER_MODE_CTC,          // Clear Timer on Compare Match
    TIMER_MODE_FAST_PWM      // Fast PWM
} TIMER_Mode_t;

/* ===================== Compare Output Modes (COM bits) ===================== */
/*
   - Non-PWM modes: DISCONNECTED / TOGGLE / CLEAR / SET
   - PWM modes: DISCONNECTED / NON-INVERTING (CLEAR) / INVERTING (SET)
*/
typedef enum {
    TIMER_OC_DISCONNECTED = 0,
    TIMER_OC_TOGGLE,
    TIMER_OC_CLEAR,
    TIMER_OC_SET
} TIMER_OCMode_t;

/* ===================== Clock / Prescaler Selection ===================== */
/* Timer0 & Timer1 share prescalers */
typedef enum {
    TIMER01_CLK_OFF = 0,  // Stopped
    TIMER01_CLK_1,
    TIMER01_CLK_8,
    TIMER01_CLK_64,
    TIMER01_CLK_256,
    TIMER01_CLK_1024,
    TIMER01_EXT_FALL,
    TIMER01_EXT_RISE
} TIMER01_Clock_t;

/* Timer2 has different prescalers on ATmega32 */
typedef enum {
    TIMER2_CLK_OFF = 0,
    TIMER2_CLK_1,
    TIMER2_CLK_8,
    TIMER2_CLK_32,
    TIMER2_CLK_64,
    TIMER2_CLK_128,
    TIMER2_CLK_256,
    TIMER2_CLK_1024
} TIMER2_Clock_t;

/* ===================== Configuration Structure ===================== */
typedef struct {
    TIMER_ID_t id;              // Timer0, Timer1, or Timer2
    TIMER_Mode_t mode;          // Normal, CTC, Fast PWM, Phase PWM
    uint8_t clock_sel;          // Use TIMER01_Clock_t or TIMER2_Clock_t
    TIMER_OCMode_t oc_mode_A;   // OC0 / OC1A / OC2
    TIMER_OCMode_t oc_mode_B;   // Only for Timer1 (OC1B); ignored for Timer0/2
    uint16_t tcnt_init;         // Counter preload
    uint16_t ocrA_init;         // OCR0 / OCR1A / OCR2 initial
    uint16_t ocrB_init;         // OCR1B initial (Timer1 only)
    uint16_t top_icr1;          // Reserved for advanced PWM with ICR1 TOP
    uint8_t int_ovf_enable;     // Overflow interrupt enable
    uint8_t int_ocA_enable;     // Compare A interrupt enable
    uint8_t int_ocB_enable;     // Compare B interrupt enable
    uint8_t configure_oc_pins;  // Auto configure OC pins
} TIMER_Config_t;

/* ===================== API ===================== */
void     TIMER_init(const TIMER_Config_t *cfg);
void     TIMER_start(TIMER_ID_t id, uint8_t clock_sel);
void     TIMER_stop(TIMER_ID_t id);
void     TIMER_setMode(TIMER_ID_t id, TIMER_Mode_t mode);
void     TIMER_setOCMode(TIMER_ID_t id, TIMER_Channel_t ch, TIMER_OCMode_t mode);
void     TIMER_setCounter(TIMER_ID_t id, uint16_t value);
uint16_t TIMER_getCounter(TIMER_ID_t id);
void     TIMER_setCompare(TIMER_ID_t id, TIMER_Channel_t ch, uint16_t value);
void     TIMER_setDutyRaw(TIMER_ID_t id, TIMER_Channel_t ch, uint8_t duty_0_255);
void     TIMER_enableInterrupts(TIMER_ID_t id, uint8_t en_ovf, uint8_t en_ocA, uint8_t en_ocB);

#endif /* TIMER_INTERFACE_H_ */

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> TIMER_program.c <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Layer: MCAL
// SWC  : TIMER (Unified)
// Target: ATmega32

#include "TIMER_interface.h"
#include "TIMER_private.h"
#include "TIMER_config.h"

/* ===== Internal helpers: apply modes / OC modes per timer ===== */

static inline void _t0_apply_mode(TIMER_Mode_t mode) {
    /* Clear WGM01:0 */
    TCCR0 &= ~((1<<WGM00) | (1<<WGM01));
    switch (mode) {
        case TIMER_MODE_NORMAL: /* 00 */ break;
        case TIMER_MODE_CTC:    TCCR0 |= (1<<WGM01); break;                /* 10 */
        case TIMER_MODE_FAST_PWM: TCCR0 |= (1<<WGM00) | (1<<WGM01); break; /* 11 */
        case TIMER_MODE_PHASE_PWM: TCCR0 |= (1<<WGM00); break;             /* 01 */
    }
}

static inline void _t2_apply_mode(TIMER_Mode_t mode) {
    TCCR2 &= ~((1<<WGM20) | (1<<WGM21));
    switch (mode) {
        case TIMER_MODE_NORMAL: /* 00 */ break;
        case TIMER_MODE_CTC:    TCCR2 |= (1<<WGM21); break;                 /* 10 */
        case TIMER_MODE_FAST_PWM: TCCR2 |= (1<<WGM20) | (1<<WGM21); break; /* 11 */
        case TIMER_MODE_PHASE_PWM: TCCR2 |= (1<<WGM20); break;              /* 01 */
    }
}

/* For Timer1 we support: NORMAL, CTC (OCR1A top), FAST_PWM 8-bit, PHASE_PWM 8-bit */
static inline void _t1_apply_mode(TIMER_Mode_t mode) {
    /* Clear WGM13..0 */
    TCCR1A &= ~((1<<WGM10) | (1<<WGM11));
    TCCR1B &= ~((1<<WGM12) | (1<<WGM13));

    switch (mode) {
        case TIMER_MODE_NORMAL: /* 0000 */ break;
        case TIMER_MODE_CTC:    /* 0100 */ TCCR1B |= (1<<WGM12); break; // OCR1A as TOP
        case TIMER_MODE_FAST_PWM:
            /* Fast PWM 8-bit: WGM13..0 = 0101 (WGM12=1, WGM10=1) */
            TCCR1B |= (1<<WGM12);
            TCCR1A |= (1<<WGM10);
            break;
        case TIMER_MODE_PHASE_PWM:
            /* Phase Correct PWM 8-bit: WGM13..0 = 0001 (WGM10=1) */
            TCCR1A |= (1<<WGM10);
            break;
    }
}

static inline void _t0_apply_ocA(TIMER_OCMode_t mode) {
    TCCR0 &= ~((1<<COM01) | (1<<COM00));
    if (mode & 0x02) TCCR0 |= (1<<COM01);
    if (mode & 0x01) TCCR0 |= (1<<COM00);
}

static inline void _t2_apply_ocA(TIMER_OCMode_t mode) {
    TCCR2 &= ~((1<<COM21) | (1<<COM20));
    if (mode & 0x02) TCCR2 |= (1<<COM21);
    if (mode & 0x01) TCCR2 |= (1<<COM20);
}

/* Timer1 has two channels */
static inline void _t1_apply_ocA(TIMER_OCMode_t mode) {
    TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0));
    if (mode & 0x02) TCCR1A |= (1<<COM1A1);
    if (mode & 0x01) TCCR1A |= (1<<COM1A0);
}
static inline void _t1_apply_ocB(TIMER_OCMode_t mode) {
    TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0));
    if (mode & 0x02) TCCR1A |= (1<<COM1B1);
    if (mode & 0x01) TCCR1A |= (1<<COM1B0);
}

/* ===== API Implementation ===== */

void TIMER_init(const TIMER_Config_t *cfg)
{
    if (!cfg) return;

    switch (cfg->id) {
    case TIMER_ID_0:
        /* Mode */
        _t0_apply_mode(cfg->mode);

        /* OC0 direction (optional) */
        if (cfg->configure_oc_pins && (cfg->oc_mode_A != TIMER_OC_DISCONNECTED)) {
            OC0_DDR |= (1<<OC0_PIN);
        }

        /* OC0 mode */
        _t0_apply_ocA(cfg->oc_mode_A);

        /* preload counter/compare (8-bit) */
        TCNT0 = (uint8_t)(cfg->tcnt_init & 0xFF);
        OCR0  = (uint8_t)(cfg->ocrA_init & 0xFF);

        /* interrupts */
        TIMER_enableInterrupts(TIMER_ID_0, cfg->int_ovf_enable, cfg->int_ocA_enable, 0);

        /* clock */
        TIMER_start(TIMER_ID_0, cfg->clock_sel);
        break;

    case TIMER_ID_1:
        _t1_apply_mode(cfg->mode);

        /* OC1A/OC1B directions (optional) */
        if (cfg->configure_oc_pins && (cfg->oc_mode_A != TIMER_OC_DISCONNECTED)) {
            OC1A_DDR |= (1<<OC1A_PIN);
        }
        if (cfg->configure_oc_pins && (cfg->oc_mode_B != TIMER_OC_DISCONNECTED)) {
            OC1B_DDR |= (1<<OC1B_PIN);
        }

        /* OC modes */
        _t1_apply_ocA(cfg->oc_mode_A);
        _t1_apply_ocB(cfg->oc_mode_B);

        /* preload counter/compare (16-bit) */
        TCNT1  = cfg->tcnt_init;
        OCR1A  = cfg->ocrA_init;
        OCR1B  = cfg->ocrB_init;
        /* ICR1 reserved for advanced modes; not used in this basic set */

        /* interrupts */
        TIMER_enableInterrupts(TIMER_ID_1, cfg->int_ovf_enable, cfg->int_ocA_enable, cfg->int_ocB_enable);

        /* clock */
        TIMER_start(TIMER_ID_1, cfg->clock_sel);
        break;

    case TIMER_ID_2:
        _t2_apply_mode(cfg->mode);

        if (cfg->configure_oc_pins && (cfg->oc_mode_A != TIMER_OC_DISCONNECTED)) {
            OC2_DDR |= (1<<OC2_PIN);
        }

        _t2_apply_ocA(cfg->oc_mode_A);

        TCNT2 = (uint8_t)(cfg->tcnt_init & 0xFF);
        OCR2  = (uint8_t)(cfg->ocrA_init & 0xFF);

        TIMER_enableInterrupts(TIMER_ID_2, cfg->int_ovf_enable, cfg->int_ocA_enable, 0);

        TIMER_start(TIMER_ID_2, cfg->clock_sel);
        break;
    }
}

void TIMER_start(TIMER_ID_t id, uint8_t clock_sel)
{
    switch (id) {
    case TIMER_ID_0:
        TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
        TCCR0 |= (clock_sel & 0x07);
        break;
    case TIMER_ID_1:
        TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
        TCCR1B |= (clock_sel & 0x07);
        break;
    case TIMER_ID_2:
        TCCR2 &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
        TCCR2 |= (clock_sel & 0x07);
        break;
    }
}

void TIMER_stop(TIMER_ID_t id)
{
    switch (id) {
    case TIMER_ID_0: TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00)); break;
    case TIMER_ID_1: TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); break;
    case TIMER_ID_2: TCCR2 &= ~((1<<CS22)|(1<<CS21)|(1<<CS20)); break;
    }
}

void TIMER_setMode(TIMER_ID_t id, TIMER_Mode_t mode)
{
    /* keep clock bits unchanged by stopping+restoring pattern if needed */
    switch (id) {
    case TIMER_ID_0: {
        uint8_t cs = TCCR0 & ((1<<CS02)|(1<<CS01)|(1<<CS00));
        TIMER_stop(TIMER_ID_0);
        _t0_apply_mode(mode);
        TCCR0 |= cs;
    } break;
    case TIMER_ID_1: {
        uint8_t cs = TCCR1B & ((1<<CS12)|(1<<CS11)|(1<<CS10));
        TIMER_stop(TIMER_ID_1);
        _t1_apply_mode(mode);
        TCCR1B |= cs;
    } break;
    case TIMER_ID_2: {
        uint8_t cs = TCCR2 & ((1<<CS22)|(1<<CS21)|(1<<CS20));
        TIMER_stop(TIMER_ID_2);
        _t2_apply_mode(mode);
        TCCR2 |= cs;
    } break;
    }
}

void TIMER_setOCMode(TIMER_ID_t id, TIMER_Channel_t ch, TIMER_OCMode_t mode)
{
    switch (id) {
    case TIMER_ID_0:
        (void)ch; _t0_apply_ocA(mode); break;
    case TIMER_ID_1:
        if (ch == TIMER_CH_A) _t1_apply_ocA(mode);
        else                  _t1_apply_ocB(mode);
        break;
    case TIMER_ID_2:
        (void)ch; _t2_apply_ocA(mode); break;
    }
}

void TIMER_setCounter(TIMER_ID_t id, uint16_t value)
{
    switch (id) {
    case TIMER_ID_0: TCNT0 = (uint8_t)value; break;
    case TIMER_ID_1: TCNT1 = value; break;
    case TIMER_ID_2: TCNT2 = (uint8_t)value; break;
    }
}

uint16_t TIMER_getCounter(TIMER_ID_t id)
{
    switch (id) {
    case TIMER_ID_0: return TCNT0;
    case TIMER_ID_1: return TCNT1;
    case TIMER_ID_2: return TCNT2;
    }
    return 0;
}

void TIMER_setCompare(TIMER_ID_t id, TIMER_Channel_t ch, uint16_t value)
{
    switch (id) {
    case TIMER_ID_0: (void)ch; OCR0 = (uint8_t)value; break;
    case TIMER_ID_1:
        if (ch == TIMER_CH_A) OCR1A = value; else OCR1B = value;
        break;
    case TIMER_ID_2: (void)ch; OCR2 = (uint8_t)value; break;
    }
}

void TIMER_setDutyRaw(TIMER_ID_t id, TIMER_Channel_t ch, uint8_t duty_0_255)
{
    /* For 8-bit Fast/Phase PWM, OCRx = duty directly.
       For Timer1 here (8-bit PWM mode), we still use 8-bit duty mapped into OCR1x[7:0]. */
    switch (id) {
    case TIMER_ID_0: (void)ch; OCR0 = duty_0_255; break;
    case TIMER_ID_1:
        if (ch == TIMER_CH_A) {
            OCR1A = duty_0_255;  // In 8-bit PWM modes, lower 8 bits are used
        } else {
            OCR1B = duty_0_255;
        }
        break;
    case TIMER_ID_2: (void)ch; OCR2 = duty_0_255; break;
    }
}

void TIMER_enableInterrupts(TIMER_ID_t id, uint8_t en_ovf, uint8_t en_ocA, uint8_t en_ocB)
{
    switch (id) {
    case TIMER_ID_0:
        if (en_ovf) TIMSK |= (1<<TOIE0); else TIMSK &= ~(1<<TOIE0);
        if (en_ocA) TIMSK |= (1<<OCIE0); else TIMSK &= ~(1<<OCIE0);
        break;
    case TIMER_ID_1:
        if (en_ovf) TIMSK |= (1<<TOIE1); else TIMSK &= ~(1<<TOIE1);
        if (en_ocA) TIMSK |= (1<<OCIE1A); else TIMSK &= ~(1<<OCIE1A);
        if (en_ocB) TIMSK |= (1<<OCIE1B); else TIMSK &= ~(1<<OCIE1B);
        break;
    case TIMER_ID_2:
        if (en_ovf) TIMSK |= (1<<TOIE2); else TIMSK &= ~(1<<TOIE2);
        if (en_ocA) TIMSK |= (1<<OCIE2); else TIMSK &= ~(1<<OCIE2);
        break;
    }
}

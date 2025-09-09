	/*
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<    TIMER_config.h    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *
 *  Author : Ahmed Shaban
 *  Layer  : MCAL
 *  SWC    : TIMER
 *
 */

#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_

/* You can place app-wide defaults here if you like. They are not enforced by code,
   but useful as documentation or compile-time selection. */

#define TIMER_DEFAULT_MODE         TIMER_MODE_FAST_PWM
#define TIMER0_DEFAULT_CLOCK       TIMER01_CLK_64
#define TIMER1_DEFAULT_CLOCK       TIMER01_CLK_64
#define TIMER2_DEFAULT_CLOCK       TIMER2_CLK_64

/* Default OC modes (non-inverting for PWM) */
#define TIMER_DEFAULT_OC_MODE_A    TIMER_OC_CLEAR
#define TIMER_DEFAULT_OC_MODE_B    TIMER_OC_CLEAR

#endif /* TIMER_CONFIG_H_ */
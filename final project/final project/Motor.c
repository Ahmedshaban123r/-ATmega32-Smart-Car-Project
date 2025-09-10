#include "Motor.h"


/*Helper : set direction output pins as outputs */
static void motor_dir_pins_inti(void)
{
	DDRD |= (1<<IN1_PIN) |(1<<IN2_PIN) |(1<<IN3_PIN) |(1<<IN4_PIN);
}

void motor_init()
{
	//timer1 configuration for fast pwm
	TIMER_Config_t t1_cfg={
		.id =TIMER_ID_1,
		.mode = TIMER_MODE_FAST_PWM_10BIT,	  // Fast PWM mode
		.clock_sel= TIMER01_CLK_8,		 // Prescaler = 8 (~2kHz PWM)
		.oc_mode_A= TIMER_OC_CLEAR,		  // Non-inverting on OC1A (PD5)
		.oc_mode_B= TIMER_OC_CLEAR,		// Non-inverting on OC1B (PD4)
		.tcnt_init=0,
		.ocrA_init=0,
		.ocrB_init=0,
		.top_icr1=1023,
		.int_ovf_enable=0,
		.int_ocA_enable=0,
		.int_ocB_enable=0,
		.configure_oc_pins=1		  // Set OC1A/OC1B pins as outputs
	};
	TIMER_init(&t1_cfg);
	
	
	/*motor pins direction*/
	motor_dir_pins_inti();
	
	motor_set_speed(0,0);
	
	 /* clear direction outputs */
	 PORTD &= ~((1<<IN1_PIN)|(1<<IN2_PIN)|(1<<IN3_PIN)|(1<<IN4_PIN));
}

void motor_set_speed(uint8_t left_spd, uint8_t right_spd)
{	 // Ensure speeds are within valid range (0-255)
	left_spd = (left_spd > 255) ? 255 : left_spd;
	right_spd = (right_spd > 255) ? 255 : right_spd;
	 // Convert 8-bit (0-255) to 10-bit (0-1023)
	 uint16_t left_10bit = (left_spd * 1023UL) / 255;
	 uint16_t right_10bit = (right_spd * 1023UL) / 255;
	TIMER_setDutyRaw(TIMER_ID_1,TIMER_CH_A, right_10bit );
	TIMER_setDutyRaw(TIMER_ID_1,TIMER_CH_B,left_10bit);
}

void motor_control(Motor_Directions dir, uint8_t left_spd, uint8_t right_spd)
{
	 /* apply speeds first (so transitions are smoother) */
	 motor_set_speed(left_spd, right_spd);

	 /* Set all direction pins at once using the direction mask */
	 PORTD = (PORTD & 0x03) | (dir & 0xFC); // Preserve PD0-PD1, set PD2-PD7 from direction
}

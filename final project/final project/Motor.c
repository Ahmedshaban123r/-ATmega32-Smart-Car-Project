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
		.mode = TIMER_MODE_FAST_PWM ,	  // Fast PWM mode
		.clock_sel= TIMER01_CLK_8,		 // Prescaler = 8 (~2kHz PWM)
		.oc_mode_A= TIMER_OC_CLEAR,		  // Non-inverting on OC1A (PD5)
		.oc_mode_B= TIMER_OC_CLEAR,		// Non-inverting on OC1B (PD4)
		.tcnt_init=0,
		.ocrA_init=0,
		.ocrB_init=0,
		.top_icr1=0,
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
{
	TIMER_setDutyRaw(TIMER_ID_1,TIMER_CH_A,right_spd);
	TIMER_setDutyRaw(TIMER_ID_1,TIMER_CH_B,left_spd);
}

void motor_control(Motor_Directions dir, uint8_t left_spd, uint8_t right_spd)
{
	 /* apply speeds first (so transitions are smoother) */
	 motor_set_speed(left_spd, right_spd);

	 /* Now set direction pins according to bit-mask in dir (nibble mapping: bit0..3 => IN1..IN4) */
	 if (dir & 0x01) PORTD |=  (1 << IN1_PIN); else PORTD &= ~(1 << IN1_PIN); // IN1
	 if (dir & 0x02) PORTD |=  (1 << IN2_PIN); else PORTD &= ~(1 << IN2_PIN); // IN2
	 if (dir & 0x04) PORTD |=  (1 << IN3_PIN); else PORTD &= ~(1 << IN3_PIN); // IN3
	 if (dir & 0x08) PORTD |=  (1 << IN4_PIN); else PORTD &= ~(1 << IN4_PIN); // IN4
}
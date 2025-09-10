#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <stdint.h>       //  uint8_t,uint16_t,uint32_t

#include "motor.h"
#include "ir.h"
#include "ULTRASONIC_interface.h"

// ============================================================================

// ============================================================================
#define TH_STOP_OBS 10
#define TH_SLOW_OBS 30

#define SPEED_NORMAL 180  // 0-255 scale
#define SPEED_SLOW   100
#define SPEED_AVOID  150
#define SPEED_SEARCH 120

#define SEARCH_TIMEOUT_MS 5000
#define SEARCH_CYCLE_TIME 1000

#define DELAY_BACKWARD_MS 500
#define DELAY_PIVOT_MS    500
#define DELAY_TURN_MS     700
#define DELAY_SEARCH_MS   400

typedef enum {
	STATE_STOP_AVOID,
	STATE_PROCESS_IR,
	STATE_SEARCH
} CarState_t;
CarState_t carState;

typedef enum {
	AVOID_SUB_STATE_START,
	AVOID_SUB_STATE_MOVE_BACKWARD,
	AVOID_SUB_STATE_CHECK_IR_AFTER_BACKWARD,
	AVOID_SUB_STATE_PIVOT_RIGHT,
	AVOID_SUB_STATE_PIVOT_LEFT,
	AVOID_SUB_STATE_ULTRASONIC_DECISION,
	AVOID_SUB_STATE_TURN_LEFT,
	AVOID_SUB_STATE_TURN_RIGHT,
	AVOID_SUB_STATE_COMPLETE
} AvoidSubState_t;
AvoidSubState_t avoidSubState = AVOID_SUB_STATE_START;

typedef enum {
	SEARCH_SUB_STATE_START,
	SEARCH_SUB_STATE_STEER_LEFT,
	SEARCH_SUB_STATE_STEER_RIGHT,
	SEARCH_SUB_STATE_COMPLETE_CYCLE,
	SEARCH_SUB_STATE_BACKUP
} SearchSubState_t;
SearchSubState_t searchSubState = SEARCH_SUB_STATE_START;

uint8_t current_driving_speed = SPEED_NORMAL;

Ultrasonic_t us_front;
Ultrasonic_t us_left;
Ultrasonic_t us_right;

typedef struct {
	uint8_t left;
	uint8_t mid;
	uint8_t right;
} IR_Readings_t;

// ============================================================================
// millis() using Timer0 (improved & atomic-safe)
// ============================================================================

TIMER_Config_t t0_cfg = {
	.id = TIMER_ID_0,
	.mode = TIMER_MODE_CTC,         // CTC mode
	.clock_sel = TIMER01_CLK_64,    // Prescaler = 64
	.oc_mode_A = TIMER_OC_DISCONNECTED, // Don’t use OC0 pin
	.oc_mode_B = TIMER_OC_DISCONNECTED, // Not valid for Timer0 anyway
	.tcnt_init = 0,
	.ocrA_init = 249,               // 16MHz / 64 / 250 = 1kHz => 1ms tick
	.ocrB_init = 0,
	.top_icr1 = 0,                  // Not used for Timer0
	.int_ovf_enable = 0,            // No overflow interrupt
	.int_ocA_enable = 1,            // Enable Compare A interrupt
	.int_ocB_enable = 0,
	.configure_oc_pins = 0          // We don’t want OC0 pin toggling
};
volatile uint32_t ms_ticks = 0;

ISR(TIMER0_COMPA_vect) {
	ms_ticks++;
}


unsigned long millis(void) {
	unsigned long ms;
	uint8_t oldSREG = SREG;
	cli();
	ms = ms_ticks;
	SREG = oldSREG;
	return ms;
}

// Non-blocking delay
unsigned char delay_non_blocking(unsigned long *prev_ms, unsigned long interval) {
	unsigned long now = millis();
	if (now - *prev_ms >= interval) {
		*prev_ms = now;
		return 1;
	}
	return 0;
}

// ============================================================================
// Initialization & sensor helpers
// ============================================================================
void sensors_init(void) {
	IR_init();

	us_front.trig_pin = PB0; us_front.echo_pin = PB1;
	us_left.trig_pin  = PB2; us_left.echo_pin  = PB3;
	us_right.trig_pin = PB4; us_right.echo_pin = PB5;

	Ultrasonic_init(us_front);
	Ultrasonic_init(us_left);
	Ultrasonic_init(us_right);
}

IR_Readings_t readIR_Sensors(void) {
	IR_Readings_t readings;
	readings.left  = IR_readLeft();
	readings.mid   = IR_readMid();
	readings.right = IR_readRight();
	return readings;
}

// ============================================================================
// Motor wrappers
// ============================================================================
void stop_car(void)         { motor_control(STOP, 0, 0); }
void move_backward(uint8_t s){ motor_control(BACKWARD, s, s); }
void pivot_right(uint8_t s)  { motor_control(RIGHT, s, s); }
void pivot_left(uint8_t s)   { motor_control(LEFT, s, s); }
void turn_right(uint8_t s)   { motor_control(RIGHT, s, s); }
void turn_left(uint8_t s)    { motor_control(LEFT, s, s); }
void go_forward(uint8_t s)   { motor_control(FORWARD, s, s); }
void steer_right(uint8_t s)  { motor_control(RIGHT, s, s); }
void steer_left(uint8_t s)   { motor_control(LEFT, s, s); }

// ============================================================================
// Avoidance non-blocking
// ============================================================================
void run_state_stop_avoid(void) {
	static unsigned long avoid_start_time = 0;
	if (avoid_start_time == 0) {
		avoid_start_time = millis();
		} else if (millis() - avoid_start_time > 5000) { // 5 second timeout
		carState = STATE_PROCESS_IR;
		avoidSubState = AVOID_SUB_STATE_START;
		avoid_start_time = 0;
		return;
	}
	static unsigned long prev_ms = 0;
	IR_Readings_t ir_readings;

	switch (avoidSubState) {
		case AVOID_SUB_STATE_START:
		stop_car();
		avoidSubState = AVOID_SUB_STATE_MOVE_BACKWARD;
		prev_ms = millis();
		break;

		case AVOID_SUB_STATE_MOVE_BACKWARD:
		move_backward(SPEED_AVOID);
		if (delay_non_blocking(&prev_ms, DELAY_BACKWARD_MS)) {
			stop_car();
			avoidSubState = AVOID_SUB_STATE_CHECK_IR_AFTER_BACKWARD;
		}
		break;

		case AVOID_SUB_STATE_CHECK_IR_AFTER_BACKWARD:
		ir_readings = readIR_Sensors();
		if (ir_readings.left == 1) {
			avoidSubState = AVOID_SUB_STATE_PIVOT_RIGHT;
			prev_ms = millis();
			} else if (ir_readings.right == 1) {
			avoidSubState = AVOID_SUB_STATE_PIVOT_LEFT;
			prev_ms = millis();
			} else {
			avoidSubState = AVOID_SUB_STATE_ULTRASONIC_DECISION;
			prev_ms = millis();
		}
		break;

		case AVOID_SUB_STATE_PIVOT_RIGHT:
		pivot_right(SPEED_AVOID);
		if (delay_non_blocking(&prev_ms, DELAY_PIVOT_MS)) {
			stop_car();
			avoidSubState = AVOID_SUB_STATE_COMPLETE;
		}
		break;

		case AVOID_SUB_STATE_PIVOT_LEFT:
		pivot_left(SPEED_AVOID);
		if (delay_non_blocking(&prev_ms, DELAY_PIVOT_MS)) {
			stop_car();
			avoidSubState = AVOID_SUB_STATE_COMPLETE;
		}
		break;

		case AVOID_SUB_STATE_ULTRASONIC_DECISION: {
			uint16_t d_left = Ultrasonic_read(us_left);
			uint16_t d_right = Ultrasonic_read(us_right);
			if (d_left == 0xFFFF && d_right == 0xFFFF) {
				avoidSubState = AVOID_SUB_STATE_TURN_RIGHT;
				} else if (d_left == 0xFFFF || d_right > d_left) {
				avoidSubState = AVOID_SUB_STATE_TURN_RIGHT;
				} else {
				avoidSubState = AVOID_SUB_STATE_TURN_LEFT;
			}
			prev_ms = millis();
			break;
		}

		case AVOID_SUB_STATE_TURN_LEFT:
		turn_left(SPEED_AVOID);
		if (delay_non_blocking(&prev_ms, DELAY_TURN_MS)) {
			stop_car();
			avoidSubState = AVOID_SUB_STATE_COMPLETE;
		}
		break;

		case AVOID_SUB_STATE_TURN_RIGHT:
		turn_right(SPEED_AVOID);
		if (delay_non_blocking(&prev_ms, DELAY_TURN_MS)) {
			stop_car();
			avoidSubState = AVOID_SUB_STATE_COMPLETE;
		}
		break;

		case AVOID_SUB_STATE_COMPLETE:
		avoidSubState = AVOID_SUB_STATE_START;
		carState = STATE_PROCESS_IR;
		break;
	}
}

// ============================================================================
// Line following
// ============================================================================
void run_state_process_ir(void) {
	IR_Readings_t ir = readIR_Sensors();
	
	// Calculate position error (-100 to +100)
	// 0 = perfectly centered, negative = left of center, positive = right of center
	int8_t error = 0;
	
	if (ir.left == 1 && ir.mid == 0 && ir.right == 1) {
		error = 0; // Perfectly centered
	}
	else if (ir.left == 0 && ir.mid == 0 && ir.right == 0) {
		error = 0; // Also centered (wide black lane)
	}
	else if (ir.right == 1) {
		error = -30; // Too far left
	}
	else if (ir.left == 1) {
		error = 30; // Too far right
	}
	else if (ir.left == 1 && ir.mid == 1) {
		error = -70; // Very far left
	}
	else if (ir.right == 1 && ir.mid == 1) {
		error = 70; // Very far right
	}

	// Proportional control: adjust motor speeds based on error
	uint8_t base_speed = current_driving_speed;
	uint8_t left_speed = base_speed + (error * 2);  // Adjust these coefficients
	uint8_t right_speed = base_speed - (error * 2); // based on testing
	
	// Ensure speeds are within limits
	left_speed = (left_speed > 255) ? 255 : left_speed;
	right_speed = (right_speed > 255) ? 255 : right_speed;
	
	motor_control(FORWARD, left_speed, right_speed);
	
	// Still handle complete loss of line
	if (ir.left == 1 && ir.mid == 1 && ir.right == 1) {
		carState = STATE_SEARCH;
		searchSubState = SEARCH_SUB_STATE_START;
	}
}

// ============================================================================
// Search zigzag
// ============================================================================
void perform_search_zigzag(uint8_t speed) {
	static unsigned long prev_ms = 0;
	static uint8_t search_cycle_count = 0;
	switch (searchSubState) {
		case SEARCH_SUB_STATE_START:
			steer_left(speed);
			prev_ms = millis();
			searchSubState = SEARCH_SUB_STATE_STEER_LEFT;
			search_cycle_count = 0;
			break;

		case SEARCH_SUB_STATE_STEER_LEFT:
			if (delay_non_blocking(&prev_ms, DELAY_SEARCH_MS)) {
				stop_car();
				searchSubState = SEARCH_SUB_STATE_STEER_RIGHT;
				prev_ms = millis();
			}
			break;

		case SEARCH_SUB_STATE_STEER_RIGHT:
			steer_right(speed);
			if (delay_non_blocking(&prev_ms, DELAY_SEARCH_MS)) {
				stop_car();
				searchSubState = SEARCH_SUB_STATE_COMPLETE_CYCLE;
			}
			break;
		case SEARCH_SUB_STATE_BACKUP:
			move_backward(SPEED_SLOW);
			if (delay_non_blocking(&prev_ms, DELAY_BACKWARD_MS)) {
				stop_car();
				searchSubState = SEARCH_SUB_STATE_STEER_LEFT;
				prev_ms = millis();
			}
			break;

		case SEARCH_SUB_STATE_COMPLETE_CYCLE:
			searchSubState = SEARCH_SUB_STATE_START;
			break;
	}
}
void run_state_search(void) {
	  static unsigned long search_start_time = 0;
	  IR_Readings_t ir = readIR_Sensors();
	  
	  // Check if we found the line during search
	  if (ir.left == 0 || ir.mid == 0 || ir.right == 0) {
		  carState = STATE_PROCESS_IR;
		  search_start_time = 0; // Reset timer
		  return;
	  }
	  
	  // Initialize search timer on first entry
	  if (search_start_time == 0) {
		  search_start_time = millis();
	  }
	  
	  // Timeout after 5 seconds of searching (adjust as needed)
	  if (millis() - search_start_time > 5000) {
		  carState = STATE_PROCESS_IR;
		  search_start_time = 0; // Reset timer
		  stop_car();
		  return;
	  }
	  
	  // Continue with the search pattern
	  perform_search_zigzag(SPEED_SEARCH);
}
// ============================================================================
// main
// ============================================================================
int main(void) {
	motor_init();
	sensors_init();
	TIMER_init(&t0_cfg);
	sei();
	carState = STATE_PROCESS_IR;

	// Non-blocking timing variables
	static unsigned long last_ultrasonic_read = 0;
	static unsigned long last_state_machine = 0;
	static uint16_t d_front = 0xFFFF;

	while (1) {
		unsigned long current_time = millis();

		// Read ultrasonic sensors every 100ms (non-blocking)
		if (current_time - last_ultrasonic_read > 100) {
			d_front = Ultrasonic_read(us_front); // This blocks for max ~60ms
			last_ultrasonic_read = current_time;
		}

		// Run state machine logic every 50ms (non-blocking)
		if (current_time - last_state_machine > 50) {
			// Only process state transitions if not in avoidance
			if (carState != STATE_STOP_AVOID) {
				if (d_front != 0xFFFF && d_front < TH_STOP_OBS) {
					carState = STATE_STOP_AVOID;
					avoidSubState = AVOID_SUB_STATE_START;
				}
				else if (d_front != 0xFFFF && d_front < TH_SLOW_OBS) {
					current_driving_speed = SPEED_SLOW;
					if (carState != STATE_SEARCH) {
						carState = STATE_PROCESS_IR;
					}
				}
				else {
					current_driving_speed = SPEED_NORMAL;
					if (carState != STATE_SEARCH) {
						carState = STATE_PROCESS_IR;
					}
				}
			}
			last_state_machine = current_time;
		}

		// Execute current state (runs continuously)
		switch (carState) {
			case STATE_STOP_AVOID:
			run_state_stop_avoid();
			break;
			case STATE_PROCESS_IR:
			run_state_process_ir();
			break;
			case STATE_SEARCH:
			run_state_search();
			break;
		}
		// Optional: Add small delay to prevent CPU overload
		_delay_ms(1);
	}
	return 0;
}

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

typedef struct {
	uint8_t trig_pin;
	uint8_t echo_pin;
} Ultrasonic_t;

void Ultrasonic_init(Ultrasonic_t us);
uint16_t Ultrasonic_read(Ultrasonic_t us);
uint8_t is_valid_distance(uint16_t distance);
#endif /* ULTRASONIC_H_ */

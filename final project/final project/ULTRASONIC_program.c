#include "ULTRASONIC_interface.h"

void Ultrasonic_init(Ultrasonic_t us) {
	DDRB |=  (1 << us.trig_pin);   // TRIG as output
	DDRB &= ~(1 << us.echo_pin);   // ECHO as input
}

uint16_t Ultrasonic_read(Ultrasonic_t us) {
	uint16_t count = 0;

	// Send trigger pulse (10us)
	PORTB &= ~(1 << us.trig_pin);
	_delay_us(2);
	PORTB |= (1 << us.trig_pin);
	_delay_us(10);
	PORTB &= ~(1 << us.trig_pin);

	// Wait for echo high
	while (!(PINB & (1 << us.echo_pin)));

	// Measure echo width
	while ((PINB & (1 << us.echo_pin))) {
		count++;
		_delay_us(1);
	}

	// Distance in cm (approx)
	return (count / 58);
}

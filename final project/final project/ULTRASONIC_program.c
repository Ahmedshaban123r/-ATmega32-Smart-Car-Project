#include "ULTRASONIC_interface.h"

void Ultrasonic_init(Ultrasonic_t us) {
	DDRB |=  (1 << us.trig_pin);   // TRIG as output
	DDRB &= ~(1 << us.echo_pin);   // ECHO as input
}

uint16_t Ultrasonic_read(Ultrasonic_t us) {
	uint16_t timeout = 0;
	uint16_t count = 0;
	// Send trigger pulse (10us)
	PORTB &= ~(1 << us.trig_pin);
	_delay_us(2);
	PORTB |= (1 << us.trig_pin);
	_delay_us(10);
	PORTB &= ~(1 << us.trig_pin);
	// Wait for echo to go high with timeout (~60ms max wait)
	timeout = 60000; // 60ms timeout
	while (!(PINB & (1 << us.echo_pin)) && timeout > 0) {
		timeout--;
		_delay_us(1);
	}
	if (timeout == 0) {
		return 0xFFFF; // Timeout - no echo detected
	}
	// Measure echo pulse width with timeout (max ~35ms for 6m distance)
	timeout = 35000; // 35ms timeout (~6 meters)
	while ((PINB & (1 << us.echo_pin)) && timeout > 0) {
		count++;
		_delay_us(1);
		timeout--;
	}
	if (timeout == 0) {
		return 0xFFFF; // Timeout - echo too long
	}
	// Distance in cm (sound travels ~34.3 cm/ms, divided by 2 for round trip)
	// count is in microseconds, so: distance = (count * 0.0343) / 2 = count / 58.3
	return (count / 58);
}
uint8_t is_valid_distance(uint16_t distance) {
	return (distance != 0xFFFF && distance > 0 && distance < 500); // Reasonable range 0-500cm
}

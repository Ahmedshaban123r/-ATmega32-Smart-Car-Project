#include "IR.h"

void IR_init(void) {
	DDRC &= ~((1 << IR_LEFT_PIN) | (1 << IR_MID_PIN) | (1 << IR_RIGHT_PIN)); // inputs
	PORTC |= (1 << IR_LEFT_PIN) | (1 << IR_MID_PIN) | (1 << IR_RIGHT_PIN);   // enable pull-ups
}

uint8_t IR_readLeft(void)  { return (PINC & (1 << IR_LEFT_PIN))  ? 1 : 0; }
uint8_t IR_readMid(void)   { return (PINC & (1 << IR_MID_PIN))   ? 1 : 0; }
uint8_t IR_readRight(void) { return (PINC & (1 << IR_RIGHT_PIN)) ? 1 : 0; }

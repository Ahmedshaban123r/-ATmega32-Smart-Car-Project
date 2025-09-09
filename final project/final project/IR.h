#ifndef IR_H_
#define IR_H_

#include <avr/io.h>

#define IR_LEFT_PIN   PC0
#define IR_MID_PIN    PC1
#define IR_RIGHT_PIN  PC2

void IR_init(void);
uint8_t IR_readLeft(void);
uint8_t IR_readMid(void);
uint8_t IR_readRight(void);

#endif /* IR_H_ */

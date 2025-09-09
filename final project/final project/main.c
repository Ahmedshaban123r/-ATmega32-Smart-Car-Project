#include "Motor.h"
#include "IR.h"
#include "ULTRASONIC_interface.h"
void sensors_init()
{
	 // IR init
	 IR_init();

	 // Ultrasonic init (3 sensors)
	 Ultrasonic_t us_front = { PB0, PB1 };
	 Ultrasonic_t us_left  = { PB2, PB3 };
	 Ultrasonic_t us_right = { PB4, PB5 };

	 Ultrasonic_init(us_front);
	 Ultrasonic_init(us_left);
	 Ultrasonic_init(us_right);
}
int main(void)
{
	motor_init();

	while (1) {
		
	}
}

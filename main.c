#include "i2c.c"
#include "mpu6050.c"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

int main(void)
{
	int16_t last_pitch_deg = 0, last_roll_deg = 0;
	char s[64];
	i2c_init();
	sei();
	if(mpu6050_init())
	{
		for(;;) ;
	}

	for(;;)
	{
		mpu6050_update();
		if(last_pitch_deg != pitch_deg || last_roll_deg != roll_deg)
		{
			last_pitch_deg = pitch_deg;
			last_roll_deg = roll_deg;
			sprintf(s, "T = %6.2f | X = %6d | Y = %6d\r\n", temperature, pitch_deg, roll_deg);

			/* Send by UART */
		}
	}

	return 0;
}

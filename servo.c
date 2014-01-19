/*
 * servo.c
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#include "servo.h"


//62500 count values

void servo_init()
{
	setPWMGenFreq(2,50);
}

#define BASE	14
#define END  	47


void servo_setPosition(int position)
{
	unsigned long int value;

		if (position >= 0 && position <= 180)
		{
			value =  BASE + (position * (END-BASE)/180);
			setSoftPWMDuty(2, value);
		}
}

void esc_setPosition(int position)
{
	unsigned long int value;

			if (position >= 0 && position <= 180)
			{
				value =  BASE + (position * (END-BASE)/180);
				setSoftPWMDuty(3, value);
			}
}

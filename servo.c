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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
	GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_1 | GPIO_PIN_0);


	PWMGenConfigure(PWM_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, 62500);
	servo_setPosition(SERVO_ZERO);
	esc_setPosition(ESC_ZERO);
	PWMOutputState(PWM_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	PWMGenEnable(PWM_BASE, PWM_GEN_1);
}

#define BASE 1560
#define END  7810

void servo_setPosition(int position)
{
	unsigned long int value;

	if (position >= 0 && position <= 180)
	{
		value =  BASE + (position * ((END-BASE)/180));
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, value);
	}
}

void esc_setPosition(int position)
{
	unsigned long int value;

	if (position >= 0 && position <= 180)
	{
		value =  BASE + (position * ((END-BASE)/180));
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, value);
	}
}

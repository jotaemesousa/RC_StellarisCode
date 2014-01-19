/*
 * servo.h
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <inc/lm3s2776.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#define ESC_MAX				140
#define ESC_ZERO			90
#define ESC_MIN				40
#define SERVO_MAX			120
#define SERVO_ZERO			90
#define SERVO_MIN			60

void servo_init();
void servo_setPosition(int position);
void esc_setPosition(int position);


#endif /* SERVO_H_ */

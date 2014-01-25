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
#include <stdint.h>
#include <stdbool.h>
#include "flash_store_values.h"

#define ESC_MAX_N20			140
#define ESC_MAX				125
#define ESC_ZERO			90
#define ESC_MIN				55
#define ESC_MIN_N20			40

#define SERVO_MAX			120
#define SERVO_MAX_PARTIAL	110
#define SERVO_ZERO			90
#define SERVO_MIN_PARTIAL	70
#define SERVO_MIN			60

void servoSetOffset(int16_t off);
bool getServoSettingStatus(void);
void setServoSettingStatus(bool status);

void servo_init();
void servo_setPosition(int position);
void esc_setPosition(int position);


#endif /* SERVO_H_ */

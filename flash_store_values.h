/*
 * flash_store_values.h
 *
 *  Created on: Jan 25, 2014
 *      Author: joao
 */

#ifndef FLASH_STORE_VALUES_H_
#define FLASH_STORE_VALUES_H_


#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include <driverlib/adc.h>
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
// debug
//#define DEBUG
//#define DEBUG_CMD



#define STELLARIS_FLASH_ADDR	0x1F000


typedef struct flash_values_
{
	int16_t servo_offset;
	int16_t para_nada;
}RC_Flash_values;

int16_t getServoOffset(void);
bool setServoOffset(int16_t offset);


#endif /* FLASH_STORE_VALUES_H_ */

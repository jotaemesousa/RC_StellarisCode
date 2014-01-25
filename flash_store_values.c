/*
 * flash_store_values.cpp
 *
 *  Created on: Jan 25, 2014
 *      Author: joao
 */

#include "flash_store_values.h"

RC_Flash_values *mini_rally_values = (RC_Flash_values *)STELLARIS_FLASH_ADDR;

int16_t getServoOffset(void)
{
	RC_Flash_values temp;
	memcpy(&temp, mini_rally_values, sizeof(temp));

	return temp.servo_offset;
}

bool setServoOffset(int16_t offset)
{
	RC_Flash_values temp;
	memcpy(&temp, mini_rally_values, sizeof(temp));

	temp.servo_offset = offset;

	FlashErase(STELLARIS_FLASH_ADDR);

	FlashProgram((unsigned long *)&temp, STELLARIS_FLASH_ADDR, sizeof(temp));

	return true;
}

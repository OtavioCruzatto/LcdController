/*
 * lcdController.c
 *
 *  Created on: Apr 5, 2023
 *      Author: Otavio
 */

#include "lcdController.h"

static void lcdSendByte(LcdController *lcdController, LcdRegisterSelection lcdRs, uint8_t byteToSend)
{
	uint8_t highNibbleOfByteToSend = ((byteToSend >> 4) & 0x0F);
	HAL_GPIO_WritePin(lcdController->db7Port, lcdController->db7Pin, (highNibbleOfByteToSend & 0x08));
	HAL_GPIO_WritePin(lcdController->db6Port, lcdController->db6Pin, (highNibbleOfByteToSend & 0x04));
	HAL_GPIO_WritePin(lcdController->db5Port, lcdController->db5Pin, (highNibbleOfByteToSend & 0x02));
	HAL_GPIO_WritePin(lcdController->db4Port, lcdController->db4Pin, (highNibbleOfByteToSend & 0x01));
	HAL_GPIO_WritePin(lcdController->rwPort, lcdController->rwPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcdController->rsPort, lcdController->rsPin, lcdRs);
	HAL_GPIO_WritePin(lcdController->enPort, lcdController->enPin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdController->enPort, lcdController->enPin, GPIO_PIN_RESET);
	HAL_Delay(1);

	uint8_t lowNibbleOfByteToSend = (byteToSend & 0x0F);
	HAL_GPIO_WritePin(lcdController->db7Port, lcdController->db7Pin, (lowNibbleOfByteToSend & 0x08));
	HAL_GPIO_WritePin(lcdController->db6Port, lcdController->db6Pin, (lowNibbleOfByteToSend & 0x04));
	HAL_GPIO_WritePin(lcdController->db5Port, lcdController->db5Pin, (lowNibbleOfByteToSend & 0x02));
	HAL_GPIO_WritePin(lcdController->db4Port, lcdController->db4Pin, (lowNibbleOfByteToSend & 0x01));
	HAL_GPIO_WritePin(lcdController->rwPort, lcdController->rwPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcdController->rsPort, lcdController->rsPin, lcdRs);
	HAL_GPIO_WritePin(lcdController->enPort, lcdController->enPin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdController->enPort, lcdController->enPin, GPIO_PIN_RESET);
	HAL_Delay(1);
}

void lcdControllerInit(LcdController *lcdController,
		uint8_t qtyOfRows, uint8_t qtyOfCharactersPerRow,
		GPIO_TypeDef *rsPort, uint16_t rsPin,
		GPIO_TypeDef *rwPort, uint16_t rwPin,
		GPIO_TypeDef *enPort, uint16_t enPin,
		GPIO_TypeDef *db4Port, uint16_t db4Pin,
		GPIO_TypeDef *db5Port, uint16_t db5Pin,
		GPIO_TypeDef *db6Port, uint16_t db6Pin,
		GPIO_TypeDef *db7Port, uint16_t db7Pin)
{
	lcdController->qtyOfRows = qtyOfRows;
	lcdController->qtyOfCharactersPerRow = qtyOfCharactersPerRow;
	lcdController->rsPort = rsPort;
	lcdController->rsPin = rsPin;
	lcdController->rwPort = rwPort;
	lcdController->rwPin = rwPin;
	lcdController->enPort = enPort;
	lcdController->enPin = enPin;
	lcdController->db4Port = db4Port;
	lcdController->db4Pin = db4Pin;
	lcdController->db5Port = db5Port;
	lcdController->db5Pin = db5Pin;
	lcdController->db6Port = db6Port;
	lcdController->db6Pin = db6Pin;
	lcdController->db7Port = db7Port;
	lcdController->db7Pin = db7Pin;

	HAL_Delay(1);
	lcdSendByte(lcdController, COMMAND, FUNCTION_SET_4BIT_MULTILINE_5X8DOTS);
	HAL_Delay(1);
	lcdSendByte(lcdController, COMMAND, CURSOR_OR_DISPLAY_SHIFT_SHIFTCURSORTOTHERIGHT);
	HAL_Delay(1);
	lcdSendByte(lcdController, COMMAND, CLEAR_DISPLAY);
	HAL_Delay(3);
	lcdSendByte(lcdController, COMMAND, RETURN_HOME);
	HAL_Delay(3);
	lcdSendByte(lcdController, COMMAND, ENTRY_MODE_SET_INCREMENT_NOTSHIFT);
	HAL_Delay(1);
	lcdSendByte(lcdController, COMMAND, DISPLAY_ON_OFF_CONTROL_DISPLAYON_CURSOROFF);
	HAL_Delay(1);
}

void lcdControllerSendString(LcdController *lcdController, LcdRow lcdRow, uint8_t *string)
{
	lcdSendByte(lcdController, COMMAND, lcdRow);

	uint8_t stringLength = strlen((char *) string);
	if(stringLength > lcdController->qtyOfCharactersPerRow)
	{
		stringLength = lcdController->qtyOfCharactersPerRow;
	}

	uint8_t charCounter = 0;
	for(charCounter = 0; charCounter < stringLength; charCounter++)
	{
		lcdSendByte(lcdController, DATA, string[charCounter]);
	}
}

void lcdControllerClearRow(LcdController *lcdController, LcdRow lcdRow)
{
	lcdSendByte(lcdController, COMMAND, lcdRow);

	uint8_t charCounter = 0;
	for(charCounter = 0; charCounter < lcdController->qtyOfCharactersPerRow; charCounter++)
	{
		lcdSendByte(lcdController, DATA, ' ');
	}
}

void lcdControllerClearDisplay(LcdController *lcdController)
{
	lcdSendByte(lcdController, COMMAND, CLEAR_DISPLAY);
}


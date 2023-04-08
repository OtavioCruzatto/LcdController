/*
 * lcdController.h
 *
 *  Created on: Apr 5, 2023
 *      Author: Otavio
 */

#ifndef INC_LCDCONTROLLER_H_
#define INC_LCDCONTROLLER_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

typedef enum LCD_COMMAND
{
	CLEAR_DISPLAY 										= 0x01,
	RETURN_HOME											= 0x02,
	ENTRY_MODE_SET_DECREMENT_NOTSHIFT 					= 0x04,
	ENTRY_MODE_SET_DECREMENT_SHIFT 						= 0x05,
	ENTRY_MODE_SET_INCREMENT_NOTSHIFT 					= 0x06,
	ENTRY_MODE_SET_INCREMENT_SHIFT 						= 0x07,
	DISPLAY_ON_OFF_CONTROL_DISPLAYOFF 					= 0x08,
	DISPLAY_ON_OFF_CONTROL_DISPLAYON_CURSOROFF 			= 0x0C,
	DISPLAY_ON_OFF_CONTROL_DISPLAYON_CURSORON_BLINKSOFF	= 0x0E,
	DISPLAY_ON_OFF_CONTROL_DISPLAYON_CURSORON_BLINKSON 	= 0x0F,
	CURSOR_OR_DISPLAY_SHIFT_SHIFTCURSORTOTHELEFT 		= 0x10,
	CURSOR_OR_DISPLAY_SHIFT_SHIFTCURSORTOTHERIGHT 		= 0x14,
	CURSOR_OR_DISPLAY_SHIFT_SHIFTDISPLAYTOTHELEFT 		= 0x18,
	CURSOR_OR_DISPLAY_SHIFT_SHIFTDISPLAYTOTHERIGHT 		= 0x1C,
	FUNCTION_SET_4BIT_SINGLELINE_5X8DOTS 				= 0x20,
	FUNCTION_SET_4BIT_MULTILINE_5X8DOTS 				= 0x28,
	FUNCTION_SET_8BIT_SINGLELINE_5X8DOTS 				= 0x30,
	FUNCTION_SET_8BIT_MULTILINE_5X8DOTS 				= 0x38
} LcdCommand;

typedef enum LCD_ROW
{
	ROW_0 = 0x80,
	ROW_1 = 0xC0,
	ROW_2 = 0x90,
	ROW_3 = 0xD0
} LcdRow;

typedef enum LCD_REGISTER_SELECTION
{
	COMMAND = 0x00,
	DATA	= 0x01
} LcdRegisterSelection;

typedef enum LCD_READ_WRITE_SELECTION
{
	WRITE = 0x00,
	READ  = 0x01
} LcdReadWriteSelection;

typedef struct
{
	uint8_t qtyOfRows;
	uint8_t qtyOfCharactersPerRow;
	GPIO_TypeDef *rsPort;
	uint16_t rsPin;
	GPIO_TypeDef *rwPort;
	uint16_t rwPin;
	GPIO_TypeDef *enPort;
	uint16_t enPin;
	GPIO_TypeDef *db4Port;
	uint16_t db4Pin;
	GPIO_TypeDef *db5Port;
	uint16_t db5Pin;
	GPIO_TypeDef *db6Port;
	uint16_t db6Pin;
	GPIO_TypeDef *db7Port;
	uint16_t db7Pin;
} LcdController;

void lcdControllerInit(LcdController *lcdController,
		uint8_t qtyOfRows, uint8_t qtyOfCharactersPerRow,
		GPIO_TypeDef *rsPort, uint16_t rsPin,
		GPIO_TypeDef *rwPort, uint16_t rwPin,
		GPIO_TypeDef *enPort, uint16_t enPin,
		GPIO_TypeDef *db4Port, uint16_t db4Pin,
		GPIO_TypeDef *db5Port, uint16_t db5Pin,
		GPIO_TypeDef *db6Port, uint16_t db6Pin,
		GPIO_TypeDef *db7Port, uint16_t db7Pin);
void lcdControllerSendString(LcdController *lcdController, LcdRow lcdRow, uint8_t *string);
void lcdControllerClearRow(LcdController *lcdController, LcdRow lcdRow);
void lcdControllerClearDisplay(LcdController *lcdController);

#endif /* INC_LCDCONTROLLER_H_ */

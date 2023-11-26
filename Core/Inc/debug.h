/*
 * debug.h
 *
 *  Created on: Apr 20, 2023
 *      Author: thean
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

#define DEBUG_ENABLED			// Uncomment to enable debugging
//#define USB_LOGGING				// Comment out DEBUG_ENABLED and uncomment USB_LOGGING to enable USB logging

int _write(int file, char *ptr, int len);

enum debug_level {
	DBG=0,		// The highest level of verbosity, used to provide detailed debugging information
	INFO=1,		// Used to provide general information about the program's execution
	WARNING=2,	// Used to indicate potential issues or unexpected behavior that should be investigated
	ERR=3,		//  Used to indicate errors that prevent the program from continuing normal execution
	CRITICAL=4,	// The highest level of severity, used to indicate critical errors that require immediate attention
	LOG_OUTPUT=5// A log level that if called, will only display certain data
};
extern enum debug_level dbg_level;	// Globally defined debug print verbosity, define in main.c
extern enum debug_level dbg;		// Variable used to pass into debug_print() when its called to define print statement verbosity, define in main.c

void debug_print(char *msg, size_t len, enum debug_level dbl);

#endif /* INC_DEBUG_H_ */

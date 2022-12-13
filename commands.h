/*
 * commands.h
 */

#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################


// ######################################### enumerations ##########################################


// ########################################## structures ###########################################


// ###################################### Global variables #########################################


// ################################### GLOBAL Function Prototypes ##################################

int xCommandProcessString(char * pCmd, bool bEcho,
		int (*Hdlr)(void *, const char *, va_list),
		void *, const char *, ...);
void vCommandProcessUART(void);

#ifdef __cplusplus
}
#endif

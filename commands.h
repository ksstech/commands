/*
 * commands.h
 */

#pragma once

#include	<stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################


// ######################################### enumerations ##########################################


// ########################################## structures ###########################################


// ###################################### Global variables #########################################


// ################################### GLOBAL Function Prototypes ##################################

int xCommandProcessString(char * pCmd, bool bEcho, int (*Hdlr)(void *, const char *, va_list), void *, const char *, ...);

#ifdef __cplusplus
}
#endif

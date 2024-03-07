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

int xCommandProcessString(char *, bool, int (*)(void *, const char *, va_list), void *, const char *, ...);

#ifdef __cplusplus
}
#endif

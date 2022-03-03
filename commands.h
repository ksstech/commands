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

int xCommandProcess(int cCmd, bool bEcho, bool ToUART, int (*Hdlr)(void *, const char *, ...), void * pV, const char * pCC, ...);

#ifdef __cplusplus
}
#endif

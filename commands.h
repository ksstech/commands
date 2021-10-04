/*
 * commands.h
 */

#pragma once

#include	"struct_union.h"							// x_time definitions stdint time

#include	<stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################

#define	cliUSE_TABLE				0

// ######################################### enumerations ##########################################


// ########################################## structures ###########################################


// ###################################### Global variables #########################################


// ################################### GLOBAL Function Prototypes ##################################

void vCommandInterpret(int cCmd, bool bEcho) ;

#ifdef __cplusplus
}
#endif

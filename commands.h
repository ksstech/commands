// commands.h

#pragma once

#include <stdarg.h>
#include "printfx.h"

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################
// ######################################### enumerations ##########################################
// ########################################## structures ###########################################

typedef struct __attribute__((packed)) command_t {
	report_t sRprt;
	u8_t *pCmd;						// command string to process
} command_t;

// ###################################### Global variables #########################################
// ################################### GLOBAL Function Prototypes ##################################

int xCommandProcess(command_t * psC);

#ifdef __cplusplus
}
#endif

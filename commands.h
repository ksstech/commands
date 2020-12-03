/*
 * commands.h
 */

#pragma once

#include	"x_struct_union.h"							// +x_time +x_definitions.h +stdint.h +time.h
#include	<stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################


// ######################################### enumerations ##########################################


// ########################################## structures ###########################################


// ###################################### Global variables #########################################

extern	const char * const FlagNames[] ;

// ################################### GLOBAL Function Prototypes ##################################


void	vIrmacosReport(void) ;
void	vGeoLocReport(void) ;

int32_t	CmndParseAddrMEM(cli_t * psCLI, uint32_t * pAddr) ;
int32_t	CmndParseAddrFLASH(cli_t * psCLI, uint32_t * pAddr) ;
int32_t	CmndParseAddrSRAM(cli_t * psCLI, uint32_t * pAddr) ;
/**
 * CmndMatch() - Scan through a table of [sub]commands trying to find matching name
 * @return	Index into the table
 */
int32_t	xCLImatch(cli_t * psCLI) ;
void	vCLIreset(cli_t * psCLI) ;
void	vCLIinit(void) ;

void	vCommandInterpret(int32_t cCmd, bool bEcho) ;

#ifdef __cplusplus
}
#endif

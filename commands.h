/*
 * commands.h
 */

#pragma once

#include	"x_struct_union.h"							// x_time x_definitions stdint time

#include	<stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################


// ######################################### enumerations ##########################################


// ########################################## structures ###########################################

typedef struct __attribute__((packed)) cli_t {
	char *			pcBeg ;								// Buffer beginning
	char *			pcStore ;							// Buffer position
	char *			pcParse ;
	struct cmnd_t *	pasList ;							// Command List
	z64_t			z64Var ;
	uint8_t			u8BSize ;
	uint8_t			u8LSize ;							// Command List Size
	uint8_t			bMode	: 1 ;						// Long mode
	uint8_t			bEcho	: 1 ;
	uint8_t			bForce	: 1 ;						// force flags display
} cli_t ;

typedef	struct	cmnd_t {
	const char	cmnd[4] ;
	int32_t	(* const hdlr) (cli_t *) ;
} cmnd_t ;

// ###################################### Global variables #########################################

extern	const char * const FlagNames[] ;

// ################################### GLOBAL Function Prototypes ##################################


void	halVARS_ReportSystem(void) ;

int32_t	CmndParseAddrMEM(cli_t * psCLI, void ** pAddr) ;
int32_t	CmndParseAddrFLASH(cli_t * psCLI, void ** pAddr) ;
int32_t	CmndParseAddrSRAM(cli_t * psCLI, void ** pAddr) ;

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

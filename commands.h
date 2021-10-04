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

typedef struct __attribute__((packed)) cli_t {
	char *			pcBeg ;								// Buffer beginning
	char *			pcStore ;							// Buffer position
	char *			pcParse ;
#if	(cliUSE_TABLE == 1)
	struct cmnd_t *	pasList ;							// Command List
#endif
	z64_t			z64Var ;
	uint8_t			u8BSize ;
#if	(cliUSE_TABLE == 1)
	uint8_t			u8LSize ;							// Command List Size
#endif
	uint8_t			bMode	: 1 ;						// Long mode
} cli_t ;

typedef	struct cmnd_t {
	const char	cmnd[4] ;
	int	(* const hdlr) (cli_t *) ;
} cmnd_t ;

// ###################################### Global variables #########################################


// ################################### GLOBAL Function Prototypes ##################################

/**
 * CmndMatch() - Scan through a table of [sub]commands trying to find matching name
 * @return	Index into the table
 */
int	xCLImatch(cli_t * psCLI) ;
void vCLIreset(cli_t * psCLI) ;
void vCLIinit(void) ;
void vCommandInterpret(int cCmd, bool bEcho) ;

#ifdef __cplusplus
}
#endif

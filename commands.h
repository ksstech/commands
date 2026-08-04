// commands.h

#pragma once

#include "report.h"

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################

// ######################################### enumerations ##########################################

// ########################################## structures ###########################################

typedef struct __attribute__((packed)) command_t {
	report_t sRprt;
	u8_t *pCmd;						// command string to process
	/* 1 = PRIVILEGED: the issuer proved either PHYSICAL access (UART console) or identity (a telnet
	 * session where xAuthenticate() actually ran and passed). Gates commands that expose secrets -
	 * currently only the 'wifi' NVS blob in 'B', which contains the PSK in clear.
	 * Deliberately NOT used to gate the recovery keys (c-R rollback, c-X PHY erase, c-Y, c-W): a
	 * mote in a reboot cycle gives seconds of access, and typing credentials before reaching a
	 * rollback would fail exactly when it matters. Emergency access stays instant and ungated;
	 * only DIAGNOSTICS that leak secrets are privileged.
	 * NOTE with ioTNETauth at its default 0 no telnet session is ever privileged, so the PSK is
	 * readable over UART only. Set ioTNETauth 1 on a mote to regain it remotely. */
	u8_t Priv;
} command_t;

// ###################################### Global variables #########################################

// ################################### GLOBAL Function Prototypes ##################################

int xCommandProcess(command_t * psC);

#ifdef __cplusplus
}
#endif

// commands.h

#pragma once

#include "report.h"

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## General MACROs #########################################

// ######################################### enumerations ##########################################

/* WHO issued the command. Used to decide how loudly a SYNTAX error is reported to the host:
 * an operator typo is not a device fault and the operator already saw the error echoed on their
 * own console, whereas a malformed command the HOST sent has nobody watching it.
 * cmdSRC_UNKNOWN is 0 so a caller that never sets the field keeps the loudest behaviour. */
enum {
	cmdSRC_UNKNOWN,					// not identified - treated as a device-level fault (ERROR)
	cmdSRC_UART,					// console, operator physically present - NOT logged to host
	cmdSRC_TNET,					// telnet session - NOTICE
	cmdSRC_RPC,						// ThingsBoard RPC, ie from the host itself - NOTICE
};

// ########################################## structures ###########################################

typedef struct __attribute__((packed)) command_t {
	report_t sRprt;
	u8_t *pCmd;						// command string to process
	/* WHO is issuing, and what they proved, in a single byte. u8Issuer = 0 is the safe default:
	 * unprivileged AND unidentified, so secrets stay hidden and syntax errors stay loud.
	 *
	 * Priv 1 = PRIVILEGED: the issuer proved either PHYSICAL access (UART console) or identity (a
	 * telnet session where xAuthenticate() actually ran and passed). Gates commands that expose
	 * secrets - currently the 'wifi' NVS blob in 'B', which contains the PSK in clear.
	 * Deliberately NOT used to gate the recovery keys (c-R rollback, c-X PHY erase, c-Y, c-W): a
	 * mote in a reboot cycle gives seconds of access, and typing credentials before reaching a
	 * rollback would fail exactly when it matters. Emergency access stays instant and ungated;
	 * only DIAGNOSTICS that leak secrets are privileged.
	 * NOTE with ioTNETauth at its default 0 no telnet session is ever privileged, so the PSK is
	 * readable over UART only. Set ioTNETauth 1 on a mote to regain it remotely. */
	union {
		struct __attribute__((packed)) {
			u8_t Priv:1;			// 1 = issuer proved physical access or identity
			u8_t Src:3;				// cmdSRC_xxx, see above - who issued this command
			u8_t spare:4;
		};
		u8_t u8Issuer;				// both fields as a single value, 0 = safe default
	};
} command_t;

// ###################################### Global variables #########################################

// ################################### GLOBAL Function Prototypes ##################################

int xCommandProcess(command_t * psC);

#ifdef __cplusplus
}
#endif

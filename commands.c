/*
 * commands.c - command interpreter
 * Copyright (c) 2017-22 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_variables.h"
#include "commands.h"

#include "actuators.h"

#if (halUSE_AEP == 1)
	#include "task_sitewhere.h"
	#include "ident1.h"
#elif (halUSE_AEP == 2)
	#include "task_thingsboard.h"
	#include "ident2.h"
#endif

#include "options.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"

#include "x_http_server.h"
#include "x_string_general.h"		// xstrncmp()
#include "x_string_to_values.h"

#include "x_errors_events.h"
#include "x_builddefs.h"
#include "x_telnet_server.h"

#include "hal_stdio.h"
#include "hal_mcu.h"				// halMCU_Report()
#include "hal_fota.h"
#include "hal_gpio.h"
#include "hal_storage.h"
#include "hal_usart.h"

#include "MQTTClient.h"				// QOSx levels

#if (configUSE_RULES > 0)
	#include "rules_decode.h"
	#include "rules_parse_text.h"
#endif

#if (halHAS_LIS2HH12 > 0)
	#include "lis2hh12.h"
#endif

#if (halHAS_LTR329ALS > 0)
	#include "ltr329als.h"
#endif

#if (halHAS_M90E26 > 0)
	#include "m90e26.h"
#endif

#if (halHAS_MCP342X > 0)
	#include "mcp342x.h"
#endif

#if (halHAS_MPL3115 > 0)
	#include "mpl3115.h"
#endif

#if (halHAS_ONEWIRE > 0)
	#include "onewire_platform.h"
	#if	(halHAS_DS18X20 > 0)
	#include "ds18x20.h"
	#endif
#endif

#if (halHAS_PCA9555 > 0)
	#include "pca9555.h"
#endif

#if (halHAS_PYCOPROC > 0)
	#include "pycoproc.h"
#endif

#if (halHAS_SI70XX > 0)
	#include "si70xx.h"
#endif

#if (halHAS_SSD1306 > 0)
	#include "ssd1306.h"
#endif

#define	debugFLAG					0xC000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Macros ################################################

#define	bufferMEMDUMP_SIZE			(1 * KILO)

// ##################################### Local variables ##########################################

static const char HelpMessage[] = {
	#ifdef ESP_PLATFORM
	"ESP32 Specific:\r\n"
	"\tc-A Boot OTA #1 FW as STA\r\n"
	#if (fotaMAX_OTA_PARTITIONS > 2)
	"\tc-B Boot OTA #2 FW as STA\r\n"
	#endif
	#if (fotaMAX_OTA_PARTITIONS > 3)
	"\tc-C Boot OTA #3 FW as STA\r\n"
	#endif
	"\tc-E delete 'syslog.txt'\r\n"
	"\tc-P switch Platform & reboot\r\n"
	"\tc-Q Toggle QOS 0->1->2->0\r\n"
	"\tc-R Revert to previous FW\r\n"
	#if (configPRODUCTION == 0)
	"\tc-T Generate WatchDog timeout\r\n"
	"\tc-U generate Invalid memory access crash\r\n"
	#endif
	"\tc-V Reboot current FW as [AP]STA (delete VARS blob)\r\n"
	"\tc-W Reboot current FW as APSTA (delete WIFI & VARS blobs)\r\n"
	#endif

	"General:\r\n"
	#if	(configPRODUCTION == 0) && (halXXX_XXX_OUT > 0)
	"ACT\t(0-x) Trigger selected actuator\r\n"
	#endif
	#if	(halXXX_XXX_OUT > 0)
	"ACT\t(A)ctuators Report\r\n"
	#endif
	#if	(configPRODUCTION == 0)
	"\t(B)lob report\r\n"
	#if	(halUSE_LITTLEFS == 1)
	"\t(C)ontent of LFS\r\n"
	#endif
	"\t(D)iagnostics\r\n"
	#if (halSOC_DIG_IN > 0)
	"GPDI\t    DigIn Pins\r\n"
	#endif
	#if	(halHAS_DS18X20 > 0)
	"1W\t    DS18X20 device info\r\n"
	#endif
	#if (halHAS_LIS2HH12 > 0)
	"lis2hh\t    lis2hh12Report\r\n"
	#endif
	#if	(halHAS_LTR329ALS > 0)
	"ltr329\t    ltr329alsReport\r\n"
	#endif
	#if	(halHAS_M90E26 > 0)
	"M90E\t    m90e26Report\r\n"
	#endif
	#if	(halHAS_MCP342X > 0)
	"MCP342x\t    mcp342xReport\r\n"
	#endif
	#if	(halHAS_MPL3115 > 0)
	"mpl3115\t    mpl3115Report\r\n"
	#endif
	#if	(halHAS_ONEWIRE > 0)
	"1W\t    Onewire info\r\n"
	#endif
	#if	(halHAS_SI70XX > 0)
	"si70xx\t    si70xxReport\r\n"
	#endif
	#if	(halHAS_SSD1306 > 0)
	"SSD1306\t    ssd1306Report\r\n"
	#endif
	#endif

	"\t(F)lags Status\r\n"
	"\t(H)elp screen display\r\n"
	#if	(configUSE_IDENT > 0)
	"\t(I)dent table\r\n"
	#endif
	"\t(L)ocation info\r\n"
	"\t(M)emory info\r\n"
	"\t(N)etwork (IP4) info\r\n"
	"\t(O)ptions display\r\n"
	#if	(configPRODUCTION == 0)
	"\t(P)artitions report\r\n"
	#endif
	"\t(R)ules display\r\n"
	"\t(S)ensors statistics\r\n"
	#if	(configPRODUCTION == 0)
	"\t(T)imer/Scatter Info\r\n"
	#endif
	"\t(U)tilization (task) statistics\r\n"
	"\t(V)erbose system info\r\n"
	"\t(W)ifi Stats\r\n"

	"Extended commands:\r\n"
	#if (halHAS_LIS2HH12 > 0)
	"mode /lis2hh12 idx ths(0-127) dur(0-255) odr(0-7) hr(0/1)\r\n"
	#endif
	#if	(halXXX_XXX_OUT > 0)
	"ACT\tdispense ch# fld# Rpt tFI tON tFO tOFF Amt\r\n"
	"ACT\tload|update ch# Rpt tFI tON tFO tOFF\r\n"
	"ACT\tadjust ch# stage# Adj\r\n"
	"ACT\tque|seq ch# S0 [... S23]]\r\n"
	#endif
	"GMAP\tioset option para1 para2\r\n"
	"GMAP\tioset 134(wifi) idx (-1 -> 3) ssid(u8 x23) pswd(u8 x23)\r\n"
	"GMAP\tioset 135(mqtt) w.x.y.z port\r\n"
	#if	(configPRODUCTION == 0)
	"GMAP\tioset 136(peek) address size\r\n"
	#endif
	"GMAP\tmode /uri para1 [para2 .. [para6]]\r\n"
	"GMAP\trule [ver] [val] IF /uri [idx] {cond} [AND/OR /uri [idx] {cond] THEN {actuation} ALSO {actuation}\r\n"
	"GMAP\tsense /uri idx Tsns Tlog [s1 [s2 [s3]]]\r\n"
	"\tregister\r\n"
	"\trestart\r\n"
	"\tshow W0 [... [W23]]\r\n"
	"\tupgrade\r\n"
	"\r\n"
} ;

// #################################### Public variables ##########################################

ubuf_t * psHB = NULL;
static u8_t cmdBuf[128]= { 0 };

static union {
	struct __attribute__((packed)) {
		u16_t esc:1;
		u16_t lsb:1;
		u16_t cli:1;
		u16_t his:1;
		u16_t idx:12;
	};
	u16_t u16;
} cmdFlag;

// ################################ Forward function declarations ##################################

void halWL_Report(void) ;
void halWL_ReportLx(void) ;
void vTaskSensorsReport(void) ;
void vControlReportTimeout(void) ;

// ############################### UART/TNET/HTTP Command interpreter ##############################

/*

 */
int	xCommandBuffer(int cCmd, bool bEcho) {
	int iRV = erSUCCESS;
	if (cCmd == CHR_ESC) {
		if ((cmdFlag.idx && cmdFlag.his) ||
			(!cmdFlag.idx && !cmdFlag.his)) {
			cmdFlag.esc = 1;							// set ESC flag
			cmdFlag.his = 0;
		} else {
			cmdFlag.u16 = 0;	// buffer NOT empty or NOT history mode, rest to default
		}
	} else if (cmdFlag.esc && cCmd == CHR_L_SQUARE) {
		cmdFlag.cli = cmdFlag.lsb = 1;					// force into CLI mode for next key
	} else if (cmdFlag.esc && cmdFlag.lsb) {
		if (cCmd == CHR_A) {							// Cursor UP
			cmdFlag.idx = xUBufStringNxt(psHB, cmdBuf, sizeof(cmdBuf));
			if (cmdFlag.idx)
				cmdFlag.his = 1;
		} else if (cCmd == CHR_B) {						// Cursor DOWN
			cmdFlag.idx = xUBufStringPrv(psHB, cmdBuf, sizeof(cmdBuf));
			if (cmdFlag.idx)
				cmdFlag.his = 1;
		} else {
			P("E=%d L=%d H=%d I=%d cCmd=%d\r\n", cmdFlag.esc, cmdFlag.lsb, cmdFlag.his, cmdFlag.idx, cCmd);
		}
		cmdFlag.esc = cmdFlag.lsb = 0;
	} else {
		if (cCmd == CHR_CR) {
			if (cmdFlag.idx) {							// CR and something in buffer?
				if (bEcho)								// yes, ....
					printfx("\r\n");
				cmdBuf[cmdFlag.idx] = 0;				// terminate command
				iRV = xRulesProcessText((char *)cmdBuf);// then execute
				if (cmdFlag.his == 0)					// if new/modified command
					vUBufStringAdd(psHB, cmdBuf, cmdFlag.idx); // save into buffer
			}
			cmdFlag.u16 = 0;
		} else if (cCmd == CHR_BS) {					// BS to remove last character
			if (cmdFlag.idx)
				--cmdFlag.idx;
		} else if (isprint(cCmd) && (cmdFlag.idx < (sizeof(cmdBuf) - 1))) {	// printable and space in buffer
			cmdBuf[cmdFlag.idx++] = cCmd;				// store character & step index
		} else {
			P("E=%d L=%d H=%d I=%d cCmd=%d\r\n", cmdFlag.esc, cmdFlag.lsb, cmdFlag.his, cmdFlag.idx, cCmd);
		}
		cmdFlag.his = 0;
	}
	if (bEcho)											// if requested
		printfx("\r\033[0K");							// clear line
	if (cmdFlag.idx) {									// anything in buffer?
		cmdFlag.cli = 1;								// ensure flag is set
		if (bEcho)										// optional refresh whole line
			printfx("%.*s \b", cmdFlag.idx, cmdBuf);
	}
	return iRV;
}

static void vCommandInterpret(int cCmd, bool bEcho) {
	int iRV = erSUCCESS;
	flagmask_t sFM;
	if (cmdFlag.cli) {
		xCommandBuffer(cCmd, bEcho);
	} else {
		switch (cCmd) {
		#if defined(ESP_PLATFORM)
		case CHR_SOH:									// c-A
		case CHR_STX:									// c-B
		case CHR_ETX: 									// c-C
			halFOTA_SetBootNumber(cCmd, fotaBOOT_REBOOT);
			break;
		case CHR_ENQ:									// c-E
			unlink("syslog.txt");
			break;
		case CHR_DC2: 									// c-R
			halFOTA_SetBootNumber(PrvPart, fotaBOOT_REBOOT);
			break;
		case CHR_DC4: 									// c-T WatchDog timeout crash
			while(1);
			break;
		case CHR_NAK:									// c-U Illegal memory write crash
			*((char *) 0xFFFFFFFF) = 1;
			break;
		case CHR_SYN:									// c-V Erase VARS blob then reboot
			halFOTA_SetBootNumber(CurPart, fotaERASE_VARS|fotaBOOT_REBOOT);
			break;
		case CHR_ETB:									// c-W Erase VARS & WIFI blobs then reboot
			halFOTA_SetBootNumber(CurPart, fotaERASE_WIFI|fotaERASE_VARS|fotaBOOT_REBOOT);
			break;
		#endif

		// ########################### Unusual (possibly dangerous) options
		#if	(configPRODUCTION == 0)
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
		case CHR_4:
		case CHR_5:
		case CHR_6:
		case CHR_7:
		#if	(halVARIANT == HW_AC00) || (halVARIANT == HW_AC01)
			cCmd -= CHR_0 ;
			vActuatorLoad(cCmd + 8, 1, 0, 6000, 0, 0);
			vActuatorLoad(cCmd, 6, 0, 500, 0, 500);
			break;

		#elif (halVARIANT == HW_WROVERKIT || halVARIANT == HW_DOITDEVKIT)
			cCmd -= CHR_0 ;
			if (cCmd < halSOC_DIG_OUT) {
				vActuatorLoad(cCmd, 5, 500, 500, 500, 500);
			} else {
				iRV = erOUT_OF_RANGE;
			}
			break;

		#elif (halVARIANT == HW_EM1P2)
			cCmd -= CHR_0;
			if (cCmd < 3) {
				m90e26Report() ;
				m90e26LoadNVSConfig(0, cCmd) ;
				m90e26LoadNVSConfig(1, cCmd) ;
				m90e26Report() ;
			} else {
				iRV = erOUT_OF_RANGE;
			}
			break ;
		#endif

		#if	(halXXX_XXX_OUT > 0)
		case CHR_A: vTaskActuatorReport(); break;
		#endif

		case CHR_B: {
			#define	blobBUFFER_SIZE			1024
			uint8_t * pBuffer = pvRtosMalloc(blobBUFFER_SIZE);
			size_t	SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_PART, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_WIFI, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_VARS, pBuffer, &SizeBlob);
			#if	(halVARIANT == HW_EM1P2)
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, pBuffer, &SizeBlob);
			#endif
			vRtosFree(pBuffer);
			break;
		}

		#if	(halUSE_LITTLEFS == 1)
		case CHR_C: halSTORAGE_InfoFS(""); break;
		#endif

		case CHR_D:
		#if (halSOC_DIG_IN > 0)
			halGPDI_Report();
		#endif
		#if	(halHAS_LIS2HH12 > 0)
			lis2hh12ReportAll();
		#endif
		#if	(halHAS_LTR329ALS > 0)
			ltr329alsReportAll();
		#endif
		#if	(halHAS_M90E26 > 0)
			m90e26Report();
		#endif
		#if	(halHAS_MCP342X > 0)
			mcp342xReportAll();
		#endif
		#if	(halHAS_MPL3115 > 0)
			mpl3115ReportAll();
		#endif
		#if	(halHAS_ONEWIRE > 0)
			OWP_Report();
		#endif
		#if	(halHAS_SI70XX > 0)
			si70xxReportAll();
		#endif
		#if	(halHAS_SSD1306 > 0)
			ssd1306Report();
		#endif
			break;
		#endif
		// ############################ Normal (non-dangerous) options
//		case CHR_E:
		case CHR_F: halVARS_ReportFlags(1); break;
//		case CHR_G:
		case CHR_H: printfx(HelpMessage); break;
		#if	(configUSE_IDENT > 0)
		case CHR_I:
			#if (halUSE_AEP == 1)
			vID1_ReportAll();
			#elif (halUSE_AEP == 2)
			vID2_ReportAll();
			#endif
			break;
		#endif
//		case CHR_J: case CHR_K:
		case CHR_L:
			halVARS_ReportGLinfo();
			halVARS_ReportTZinfo();
			break;
		case CHR_M:
			sFM.u32Val = makeMASK11x21(1,0,0,1,1,1,1,1,1,1,1,0);
			vRtosReportMemory(NULL, 0, sFM);
			break;
		case CHR_N: xNetReportStats(); break;
		case CHR_O: vOptionsShow(); break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_P: halFOTA_ReportPartitions(); break ;
		#endif
//		case CHR_Q:
		case CHR_R: vRulesDecode(); break;
		case CHR_S: vTaskSensorsReport(); break;
		#if	(configPRODUCTION == 0)
		case CHR_T: vSysTimerShow(0xFFFFFFFF); break;
		#endif
		case CHR_U:
			sFM.u32Val = makeMASK09x23(0,1,1,1,1,1,1,1,1,0x007FFFFF);
			xRtosReportTasks(NULL, 0, sFM);
			break;
		case CHR_V:
			halMCU_Report();
			halVARS_ReportFirmware();
			halWL_ReportLx();
			vSyslogReport();
			IF_EXEC_0(configCONSOLE_HTTP == 1, vHttpReport);
			IF_EXEC_0(configCONSOLE_TELNET == 1, vTnetReport);
			#if	(halUSE_AEP == 1)
			#include "task_sitewhere.h"
			vSW_Report() ;
			#elif (halUSE_AEP == 2)
			#include "task_thingsboard.h"
			vTB_Report();
			#endif
			halVARS_ReportSystem();
			vControlReportTimeout();
			vUBufReport(psHB);
			break ;
		case CHR_W: halWL_Report(); break;
//		case CHR_X: case CHR_Y: case CHR_Z:
		default: xCommandBuffer(cCmd, bEcho);
		}
	}
	if (iRV < erSUCCESS)
		xSyslogError(__FUNCTION__, iRV);
}

/**
 * @brief	process a command string and call the [optional handler] to process the buffered output
 */
int xCommandProcessString(char * pCmd, bool bEcho, int (*Hdlr)(void *, const char *, va_list), void * pV, const char * pCC, ...) {
	xStdioBufLock(portMAX_DELAY);
	if (psHB == NULL) {
		psHB = psUBufCreate(NULL, NULL, (ioB4GET(ioCLIbuf)+1) << 7, 0);
		psHB->f_history = 1;
	}
	int iRV = 0;
	while (*pCmd) {
		halVARS_ReportFlags(0);							// handle flag changes since previously here
		vCommandInterpret(*pCmd++, bEcho);				// process it..
		++iRV;
	}
	if (iRV > 1)
		vCommandInterpret(CHR_CR, bEcho);
	halVARS_CheckChanges();								// handle VARS if changed
	halVARS_ReportFlags(0);								// report flags if changed
	if (Hdlr) {
		va_list vaList;
		va_start(vaList, pCC);
		iRV = Hdlr(pV, pCC, vaList);					// empty buffer
		va_end(vaList);
	}
	xStdioBufUnLock();
	return iRV;
}

int vCommandEmptyBuffer(void * pV, const char * pCC, va_list vaList) {
	int iRV = 0;
	if (allSYSFLAGS(sfU0ACTIVE << configSTDIO_UART_CHAN)) {
		while (xStdioBufAvail()) {
			putcharRT(xStdioBufGetC());
			++iRV;
		}
	}
	return iRV;
}

void vCommandProcessUART(void) {
	char caChr[2];
	int iRV = halUART_GetChar(configSTDIO_UART_CHAN);
	caChr[0] = (iRV == EOF) ? 0 : iRV;
	caChr[1] = 0;
	xCommandProcessString(caChr, 1, vCommandEmptyBuffer, NULL, NULL);
}

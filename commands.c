/*
 * commands.c - command interpreter
 * Copyright (c) 2017-22 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#include "actuators.h"
#include "commands.h"
#include "task_aep.h"
#include "identity.h"

#include "printfx.h"
#include "rules.h"					// xRulesProcessText
#include "statistics.h"
#include "syslog.h"
#include "systiming.h"

#include "x_builddefs.h"
#include "x_errors_events.h"
#if (includeHTTP_TASK > 0)
	#include "x_http_server.h"
#endif
#include "x_string_to_values.h"
#if (includeTNET_TASK > 0)
	#include "tnet_server.h"
#endif

#include "hal_device_includes.h"
#if (halUSE_I2C > 0)
	#include "hal_i2c_common.h"
#endif

#include "hal_network.h"
#include "hal_stdio.h"
#include "hal_mcu.h"				// halMCU_Report()
#include "hal_fota.h"
#include "hal_storage.h"
#include "hal_usart.h"

#include "MQTTClient.h"				// QOSx levels

#if (appUSE_RULES > 0)
	#include "rules.h"
#endif

#if (halHAS_M90E26 > 0)
	#include "m90e26.h"
#endif

#define	debugFLAG					0xF000

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
	"\tc-A/B/C Boot OTA #1/2/3 FW as STA\r\n"
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
	"\t(D)iagnostics ["
		#if (halSOC_DIG_IN > 0)
		"gpi\t"
		#endif
		#if (halHAS_ADE7953 > 0)
		"ade7953\t"
		#endif
		#if	(halHAS_DS18X20 > 0)
		"ds18x20\t"
		#endif
		#if (halHAS_LIS2HH12 > 0)
		"lis2hh\t"
		#endif
		#if	(halHAS_LTR329ALS > 0)
		"ltr329\t"
		#endif
		#if	(halHAS_M90E26 > 0)
		"m90e26\t"
		#endif
		#if	(halHAS_MCP342X > 0)
		"mcp342x\t"
		#endif
		#if	(halHAS_MPL3115 > 0)
		"mpl3115\t"
		#endif
		#if	(halHAS_ONEWIRE > 0)
		"1Wire\t"
		#endif
		#if (halHAS_PCA9555 > 0)
		"pca9555\t"
		#endif
		#if (halHAS_PCF8574 > 0)
		"pcf8574\t"
		#endif
		#if	(halHAS_SI70XX > 0)
		"si70xx\t"
		#endif
		#if	(halHAS_SSD1306 > 0)
		"ssd1306\t"
		#endif
	"]\r\n"
	#endif

	"\t(F)lags Status\r\n"
	"\t(H)elp screen display\r\n"
	#if	(appUSE_IDENT > 0)
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
	"\treboot | register | upgrade | show W0 [... [W23]]\r\n"
	#if	(halXXX_XXX_OUT > 0)
	"ACT\tdispense ch# fld# Rpt tFI tON tFO tOFF Amt\r\n"
	"ACT\tload|update ch# Rpt tFI tON tFO tOFF\r\n"
	"ACT\tadjust ch# stage# Adj\r\n"
	"ACT\tque|seq ch# S0 [... S23]]\r\n"
	#endif
	"GMAP\tioset option para1 para2\r\n"
	"GMAP\tioset 141(nwmo) {0->3} off/sta/ap/sta+ap\r\n"
	"GMAP\tioset 142(wifi) idx (-1 -> 3) ssid(u8 x23) pswd(u8 x23)\r\n"
	"GMAP\tioset 143(mqtt) w.x.y.z port\r\n"
	#if	(configPRODUCTION == 0)
		"GMAP\tioset 144(peek) address size\r\n"
		"GMAP\tioset 145(poke) address size\r\n"
	#endif

	"GMAP\tmode /uri para1 [para2 .. [para6]]\r\n"
		#if (halHAS_ADE7953 > 0)
		"\tmode /ade7954 idx ???\r\n"
		#endif
		#if	(halHAS_DS18X20 > 0)
		"\tmode /ds18x20 idx lo=-128~127 hi=-128~127 res=9~12 wr=0/1\r\n"
		#endif
		#if	(halHAS_LIS2HH12 > 0)
		"\tmode /lis2hh12 0 ths(0-127) dur(0-255)\r\n"
		"\tmode /lis2hh12 1 hr(0/1) odr(0-7) bdu(0/1) ?en(0->7)\r\n"
		"\tmode /lis2hh12 3 CTRL1  IG_CFG1  \r\n"
		"\tmode /lis2hh12 4 bw(0->3) fs(0->3) bw(0/1) Aincr(0/1)\r\n"
		#endif
		#if	(halHAS_LTR329ALS > 0)
		"\tmode /ltr329als idx gain=0~3/6/7 time=0~7 rate=0~7\r\n"
		#endif
		#if	(halHAS_M90E26 > 0)
		"\tmode /m90e26 idx 1=gainL val=1/4/8/16/24\r\n"
		#if	(m90e26NEUTRAL > 0)
		"\t\t2=gainN val=1/2/4\r\n"
		#endif
		"\t\t4=reCalib\r\n"
		"\t\t5=Calc CurOfst\r\n"
		"\t\t6=Calc PwrOfst\r\n"
		"\t\t7=Save pos=0~" mySTRINGIFY(m90e26CALIB_NUM-1) " 'Calibration Data'\r\n"
		"\t\t8=Delete 'ALL Calibration data'\r\n"
		"\t\t9=WriteReg reg=" mySTRINGIFY(SOFTRESET) "~" mySTRINGIFY(CRC_2) " val=0~0xFFFF\r\n"
		#endif
		#if (configPRODUCTION == 0) && (halHAS_MB_ACT > 0 || halHAS_MB_SEN > 0)
		"\tcmd /mb TBC\r\n"
		#endif
		#if (halHAS_MB_ACT > 0)
		"\tcmd /mb/act \r\n"
		"\tmode /mb/act idx TBC\r\n"
		#endif
		#if (halHAS_MB_SEN > 0)
		"\tcmd /mb/sen  \r\n"
		"\tmode /mb/sen idx TBC\r\n"
		#endif
		#if	(halHAS_MCP342X > 0)
		"\tmode /mcp342x idx TBC\r\n"
		#endif
		#if	(halHAS_MPL3115 > 0)
		"\tmode /mpl3115 idx TBC\r\n"
		#endif
		#if	(halHAS_PYCOPROC > 0)
		"\tmode /pycoproc idx TBC\r\n"
		#endif
		#if	(halHAS_SI70XX > 0)
		"\tmode /si70xx idx TBC\r\n"
		#endif
		#if (halSOC_DIG_IN > 0)
		"\tmode /gdi idx inv=0~1 type=0~5 dly=0~255\r\n"
		#endif
		#if	(halSOC_ANA_IN > 0)
		"\tmode /gai idx attn=0~11db width=9~11]\r\n"
		#endif
		#if	(halXXX_XXX_OUT > 0)
		"\tmode /act idx TBC\r\n"
		#endif
	"GMAP\tsense /uri idx Tsns Tlog [s1 [s2 [s3]]]\r\n"
	"GMAP\trule [ver] [val] IF /uri [idx] {cond} [AND/OR /uri [idx] {cond] THEN {actuation} ALSO {actuation}\r\n"
	strCRLF
};

// #################################### Public variables ##########################################

static ubuf_t * psHB = NULL;
static u8_t cmdBuf[128]= { 0 };

static union {
	struct __attribute__((packed)) {
		u16_t esc:1;
		u16_t lsb:1;				// 1=Left Square Bracket (LSB) received, multi char key start
		u16_t cli:1;				// 1= non single char UC command buffer mode
		u16_t his:1;				// in HIStory mode, scrolling up (A) or down (B)
		u16_t idx:12;				// slot in buffer where next key to be stored, 0=empty
	};
	u16_t u16;
} cmdFlag;

// ############################### UART/TNET/HTTP Command interpreter ##############################

void xCommandReport(report_t * psR, int cCmd) {
	wprintfx(psR, "E=%d L=%d H=%d I=%d cCmd=%d\r\n", cmdFlag.esc, cmdFlag.lsb, cmdFlag.his, cmdFlag.idx, cCmd);
}

/*
 * @brief
 */
int	xCommandBuffer(report_t * psR, int cCmd, bool bEcho) {
	int iRV = erSUCCESS;
	if (cCmd == CHR_ESC) {
		if ((cmdFlag.idx && cmdFlag.his) || (cmdFlag.idx == 0 && cmdFlag.his == 0)) {
			cmdFlag.esc = 1;		// set ESC flag
			cmdFlag.his = 0;
		} else {
			cmdFlag.u16 = 0;		// buffer NOT empty or NOT history mode, reset to default
		}

	} else if (cmdFlag.esc && cCmd == CHR_L_SQUARE) {
		cmdFlag.lsb = 1;			// Left Square Bracket received, set flag
		cmdFlag.cli = 1;			// force into CLI mode for next key

	} else if (cmdFlag.esc && cmdFlag.lsb) {
		// ESC[ received, next code is extended/function key....
		if (cCmd == CHR_A) {							// Cursor UP
			cmdFlag.idx = xUBufStringNxt(psHB, cmdBuf, sizeof(cmdBuf));
			if (cmdFlag.idx) {
				cmdFlag.his = 1;
			}
		} else if (cCmd == CHR_B) {						// Cursor DOWN
			cmdFlag.idx = xUBufStringPrv(psHB, cmdBuf, sizeof(cmdBuf));
			if (cmdFlag.idx) {
				cmdFlag.his = 1;
			}
		} else if (cCmd == CHR_C) {						// Cursor RIGHT
			//
		} else if (cCmd == CHR_D) {						// Cursor LEFT
			//
		} else {
			xCommandReport(psR, cCmd);
		}
		cmdFlag.lsb = 0;
		cmdFlag.esc = 0;

	} else {
		if (cCmd == CHR_CR) {
			if (cmdFlag.idx) {							// CR and something in buffer?
				cmdBuf[cmdFlag.idx] = 0;				// terminate command
				wprintfx(psR, "\r\n");
				iRV = xRulesProcessText((char *)cmdBuf);// then execute
				if (cmdFlag.his == 0) {					// if new/modified command
					vUBufStringAdd(psHB, cmdBuf, cmdFlag.idx); // save into buffer
				}
			}
			cmdFlag.u16 = 0;

		} else if (cCmd == CHR_BS || cCmd == CHR_DEL) {	// BS (macOS screen DEL) to remove previous character
			if (cmdFlag.idx) {							// yes,
				--cmdFlag.idx;							// step 1 slot back
				if (cmdFlag.idx == 0) {					// if buffer now empty
					cmdFlag.u16 = 0;					// reset to default (non cli/history) mode
				}
			}

		} else if (isprint(cCmd) && (cmdFlag.idx < (sizeof(cmdBuf) - 1))) {	// printable and space in buffer
			cmdBuf[cmdFlag.idx++] = cCmd;				// store character & step index

		} else if (cCmd != CHR_LF) {
			xCommandReport(psR, cCmd);
		}
		cmdFlag.his = 0;
	}
	if (bEcho) {										// if requested
		wprintfx(psR, "\r\033[0K");						// clear line
	}
	if (cmdFlag.idx) {									// anything in buffer?
		cmdFlag.cli = 1;								// ensure flag is set
		if (bEcho) {									// optional refresh whole line
			wprintfx(psR, "%.*s \b", cmdFlag.idx, cmdBuf);
		}
	}
	return iRV;
}

static int vCommandEmptyBuffer(void * pV, const char * pCC, va_list vaList) {
	int iRV = 0;
	if (allSYSFLAGS(sfU0ACTIVE << configSTDIO_UART_CHAN))
		while (xStdioBufAvail()) { putcharRT(xStdioBufGetC()); ++iRV; }
	return iRV;
}

// ################################# command string/character support ##############################

static void vCommandInterpret(int cCmd, bool bEcho) {
	int iRV = erSUCCESS;
	report_t sRprt = { 0 };
	if (cmdFlag.cli) {
		xCommandBuffer(&sRprt, cCmd, bEcho);
	} else {
		clrSYSFLAGS(sfKEY_EOF);
		switch (cCmd) {	// CHR_E CHR_G CHR_J CHR_K CHR_Q CHR_X CHR_Y CHR_Z
		#if defined(ESP_PLATFORM)
		case CHR_SOH:															// c-A
		case CHR_STX:															// c-B
		case CHR_ETX: halFOTA_SetBootNumber(cCmd, fotaBOOT_REBOOT); break;		// c-C
		case CHR_ENQ: unlink("syslog.txt"); break;								// c-E
		case CHR_DC2: halFOTA_SetBootNumber(PrvPart, fotaBOOT_REBOOT); break;	// c-R
		case CHR_DC4: while(1); break;					// c-T WatchDog timeout crash
		case CHR_NAK: *((char *)0xFFFFFFFF)=1; break;	// c-U Illegal memory write crash
		case CHR_SYN:									// c-V Erase VARS blob then reboot
			halFOTA_SetBootNumber(CurPart, fotaERASE_VARS|fotaBOOT_REBOOT);
			break;
		case CHR_ETB:				// c-W Erase VARS,WIFI M90E26/ADE7953 blobs then reboot
			halFOTA_SetBootNumber(CurPart, fotaERASE_WIFI|fotaERASE_VARS|fotaERASE_DEVNVS|fotaBOOT_REBOOT);
			break;
		#endif

		// ########################### Unusual (possibly dangerous) options
		case CHR_SUB: setSYSFLAGS(sfKEY_EOF); break;

		#if	(configPRODUCTION == 0)
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
		case CHR_4:
		case CHR_5:
		case CHR_6:
		case CHR_7:
			cCmd -= CHR_0;
			#if (cmakePLTFRM == HW_EM1P2)
			if (cCmd < 3) {
				m90e26LoadNVSConfig(0, cCmd);
				m90e26LoadNVSConfig(1, cCmd);
			} else
			#elif (cmakePLTFRM == HW_EM3P2)
			if (cCmd < 3) {
				m90e36Report();
				m90e36LoadNVSConfig(0, cCmd);
				m90e36LoadNVSConfig(1, cCmd);
				m90e36Report();
			} else
			#elif (cmakePLTFRM==HW_AC00 || cmakePLTFRM==HW_AC01 || cmakePLTFRM==HW_DK41 || cmakePLTFRM==HW_KC868A4 || cmakePLTFRM==HW_KC868A6 || cmakePLTFRM==HW_SP1PM || cmakePLTFRM==HW_SP2PM)
			if (cCmd < halXXX_DIG_OUT) {
				vActuatorLoad(cCmd, 5, 0, 500, 0, 500);
				#if	(cmakePLTFRM == HW_AC00 || cmakePLTFRM == HW_AC01)
				vActuatorLoad(cCmd + 8, 1, 0, 6000, 0, 0);
				#endif
			} else
			#endif
			{	iRV = erOUT_OF_RANGE; }
			break;

		#if	(halXXX_XXX_OUT > 0)
		case CHR_A: vTaskActuatorReport(&sRprt); break;
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
			#if	(cmakePLTFRM == HW_EM1P2)
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, m90e26STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			#if	(cmakePLTFRM == HW_SP2PM)
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, ade7953STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			vRtosFree(pBuffer);
			break;
		}

		#if	(halUSE_LITTLEFS == 1)
		case CHR_C: halSTORAGE_InfoFS(NULL, ""); break;
		#endif

		case CHR_D:
			sRprt.sFM.aNL = 1; sRprt.sFM.aColor = 1;
			#if (halSOC_ANA_IN > 0)
			halGAI_Report(&sRprt);
			#endif
			#if (halSOC_ANA_OUT > 0)
			halGAO_Report(&sRprt);
			#endif
			#if (halSOC_DIG_IN > 0)
			halGDI_Report(&sRprt);
			#endif
			#if	(halHAS_ADE7953 > 0)
			ade7953Report(NULL);
			#endif
			#if	(halHAS_DS1307 > 0)
			ds1307Report(NULL, strNUL);
			#endif
			#if	(halHAS_LIS2HH12 > 0)
			lis2hh12ReportAll(&sRprt);
			#endif
			#if	(halHAS_LTR329ALS > 0)
			ltr329alsReportAll(&sRprt);
			#endif
			#if	(halHAS_M90E26 > 0)
			m90e26Report(&sRprt);
			#endif
			#if	(halHAS_MCP342X > 0)
			mcp342xReportAll(&sRprt);
			#endif
			#if	(halHAS_MPL3115 > 0)
			mpl3115ReportAll(&sRprt);
			#endif
			#if	(halHAS_ONEWIRE > 0)
			OWP_Report(&sRprt);
			#endif
			#if (halHAS_PCA9555 > 0)
			pca9555Report(&sRprt);
			#endif
			#if (halHAS_PCF8574 > 0)
			pcf8574Report(&sRprt);
			#endif
			#if (halHAS_PYCOPROC > 0)
			pycoprocReportAll(&sRprt);
			#endif
			#if	(halHAS_SI70XX > 0)
			si70xxReportAll(&sRprt);
			#endif
			#if	(halHAS_SSD1306 > 0)
			ssd1306Report(&sRprt);
			#endif
			halWL_TimeoutReport(&sRprt);
			vUBufReport(psHB);
			break;
		#endif						// (configPRODUCTION == 0)

		// ############################ Normal (non-dangerous) options
		case CHR_F: halVARS_ReportFlags(&sRprt, 1); break;
		case CHR_H: printfx(HelpMessage); break;

		case CHR_I:
			#if	(appUSE_IDENT > 0)
			vID_Report();
			#else
			printfx("No identity support\r\n");
			#endif
			break;

		case CHR_L: halVARS_ReportGLinfo(&sRprt); halVARS_ReportTZinfo(&sRprt); break;

		case CHR_M:
			sRprt.sFM = (fm_t) makeMASK09x23(0,0,0,0,0,0,0,1,1, 0x0000FC0F);
			xRtosReportMemory(&sRprt);
			break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_N: xNetReportStats(&sRprt); break;
		#endif

		case CHR_O: vOptionsShow(); break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_P: halFOTA_ReportPartitions(); break ;
		#endif

		case CHR_R: vRulesDecode(); break;

		case CHR_S:
			int xTaskSensorsReport(report_t *);
			sRprt.sFM = (fm_t) makeMASK12x20(1,1,1,1,1,1,1,1,1,1,1,1, 0x000FFFFF);
			xTaskSensorsReport(&sRprt);
			break;

		#if	(configPRODUCTION == 0)
		case CHR_T: vSysTimerShow(&sRprt, 0xFFFFFFFF); break;
		#endif

		case CHR_U:
			sRprt.sFM = (fm_t) makeMASK09x23(1,1,1,1,1,1,1,1,0, 0x007FFFFF);
			xRtosReportTasks(&sRprt);
			break;

		case CHR_V:
			halMCU_Report(&sRprt);
			halWL_ReportLx(&sRprt);
			vSyslogReport(&sRprt);
			#if (includeHTTP_TASK > 0)
			vHttpReport(&sRprt);
			#endif
			#if (includeTNET_TASK > 0)
			vTnetReport(&sRprt);
			#endif
			#if (halHAS_MB_SEN > 0 || halHAS_MB_ACT > 0)
			xEpMBC_ClientReport(&sRprt);
			#endif

			#if (cmakeAEP > 0)
			sRprt.sFM = (fm_t) makeMASK09x23(1,1,1,1,1,1,1,1,1, 0x0);
			vAEP_Report(&sRprt);
			#endif
			halVARS_ReportApp(&sRprt);
			break;

		case CHR_W: halWL_Report(&sRprt); break;
		default: xCommandBuffer(&sRprt, cCmd, bEcho);
		}
	}
	if (iRV < erSUCCESS) xSyslogError(__FUNCTION__, iRV);
}

/**
 * @brief	process a command string and call the [optional handler] to process the buffered output
 * @param	pCmd - command string to process, can be single character null terminated
 * @param	bEcho - if true all command characters will be echo'd
 * @param	Hdlr - pointer to function to empty buffer once processing done
 * @param	pV - context pointer as parameter 1 for handler
 * @param	pCC - printf style format control string
 * @param	... - optional list of parameters as required by format string
 * @return	number of characters passed to output
 */
int xCommandProcessString(char * pCmd, bool bEcho, int (*Hdlr)(void *, const char *, va_list), void * pV, const char * pCC, ...) {
	report_t sRprt = { 0 };
	xStdioBufLock(portMAX_DELAY);
	if (psHB == NULL) {
		psHB = psUBufCreate(NULL, NULL, (ioB4GET(ioCLIbuf)+1) << 7, 0);
		psHB->f_history = 1;
	}
	int iRV = 0;
	while (*pCmd) {
		halVARS_ReportFlags(&sRprt, 0);					// handle flag changes since previously here
		vCommandInterpret(*pCmd++, bEcho);				// process it..
		++iRV;
	}
	if (iRV > 1) vCommandInterpret(CHR_CR, bEcho);
	halVARS_CheckChanges();								// handle VARS if changed
	halVARS_ReportFlags(&sRprt, 0);						// report flags if changed
	if (Hdlr) {
		va_list vaList;
		va_start(vaList, pCC);
		iRV = Hdlr(pV, pCC, vaList);					// empty buffer
		va_end(vaList);
	}
	xStdioBufUnLock();
	return iRV;
}

// ######################################## UART specific support ##################################

void vCommandProcessUART(void) {
	char caChr[2];
	int iRV = halUART_GetChar(configSTDIO_UART_CHAN);
	caChr[0] = (iRV == EOF) ? 0 : iRV;
	caChr[1] = 0;
	xCommandProcessString(caChr, 1, vCommandEmptyBuffer, NULL, NULL);
}

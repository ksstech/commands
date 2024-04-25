// commands.c - Copyright (c) 2017-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"
#include "hal_device_includes.h"
#if (halUSE_I2C > 0)
	#include "hal_i2c_common.h"
#endif
#include "hal_fota.h"
#include "hal_gpio.h"
#include "hal_mcu.h"				// halMCU_Report()
#include "hal_memory.h"
#include "hal_network.h"
#include "hal_options.h"
#include "hal_stdio.h"
#include "hal_storage.h"
#include "hal_usart.h"

#include "actuators.h"
#include "commands.h"
#include "identity.h"
#include "task_aep.h"
#include "task_sensors.h"

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
#include "MQTTClient.h"				// QOSx levels

#if (appUSE_RULES > 0)
	#include "rules.h"
#endif

// ######################################## Macros ################################################

#define	debugFLAG					0xF000
#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

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
	#if	(configPRODUCTION == 0) && (HAL_XXO > 0)
	"ACT\t(0-x) Trigger selected actuator\r\n"
	#endif
	#if	(HAL_XXO > 0)
	"ACT\t(A)ctuators Report\r\n"
	#endif
	#if	(configPRODUCTION == 0)
	"\t(B)lob report\r\n"
	#if	(halUSE_LITTLEFS == 1)
	"\t(C)ontent of LFS\r\n"
	#endif
	"\t(D)iagnostics ["
		#if (HAL_GDI > 0)
		"gpi\t"
		#endif
		#if (HAL_ADE7953 > 0)
		"ade7953\t"
		#endif
		#if	(HAL_DS18X20 > 0)
		"ds18x20\t"
		#endif
		#if (HAL_LIS2HH12 > 0)
		"lis2hh\t"
		#endif
		#if	(HAL_LTR329ALS > 0)
		"ltr329\t"
		#endif
		#if	(HAL_M90E26 > 0)
		"m90e26\t"
		#endif
		#if	(HAL_MCP342X > 0)
		"mcp342x\t"
		#endif
		#if	(HAL_MPL3115 > 0)
		"mpl3115\t"
		#endif
		#if	(HAL_ONEWIRE > 0)
		"1Wire\t"
		#endif
		#if (HAL_PCA9555 > 0)
		"pca9555\t"
		#endif
		#if (HAL_PCF8574 > 0)
		"pcf8574\t"
		#endif
		#if	(HAL_SI70XX > 0)
		"si70xx\t"
		#endif
		#if	(HAL_SSD1306 > 0)
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
	#if	(HAL_XXO > 0)
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
		#if (HAL_ADE7953 > 0)
		"\tmode /ade7953 idx ???\r\n"
		#endif
		#if	(HAL_DS18X20 > 0)
		"\tmode /ds18x20 idx lo=-128~127 hi=-128~127 res=9~12 wr=0/1\r\n"
		#endif
		#if	(HAL_LIS2HH12 > 0)
		"\tmode /lis2hh12 0 ths(0-127) dur(0-255)\r\n"
		"\tmode /lis2hh12 1 hr(0/1) odr(0-7) bdu(0/1) ?en(0->7)\r\n"
		"\tmode /lis2hh12 3 CTRL1  IG_CFG1  \r\n"
		"\tmode /lis2hh12 4 bw(0->3) fs(0->3) bw(0/1) Aincr(0/1)\r\n"
		#endif
		#if	(HAL_LTR329ALS > 0)
		"\tmode /ltr329als idx gain=0~3/6/7 time=0~7 rate=0~7\r\n"
		#endif
		#if	(HAL_M90E26 > 0)
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
		#if (configPRODUCTION == 0) && (HAL_MB_ACT > 0 || HAL_MB_SEN > 0)
		"\tcmd /mb TBC\r\n"
		#endif
		#if (HAL_MB_ACT > 0)
		"\tcmd /mb/act \r\n"
		"\tmode /mb/act idx TBC\r\n"
		#endif
		#if (HAL_MB_SEN > 0)
		"\tcmd /mb/sen  \r\n"
		"\tmode /mb/sen idx TBC\r\n"
		#endif
		#if	(HAL_MCP342X > 0)
		"\tmode /mcp342x idx TBC\r\n"
		#endif
		#if	(HAL_MPL3115 > 0)
		"\tmode /mpl3115 idx TBC\r\n"
		#endif
		#if	(HAL_PYCOPROC > 0)
		"\tmode /pycoproc idx TBC\r\n"
		#endif
		#if	(HAL_SI70XX > 0)
		"\tmode /si70xx idx TBC\r\n"
		#endif
		#if (HAL_GDI > 0)
		"\tmode /gdi idx inv=0~1 type=0~5 dly=0~255\r\n"
		#endif
		#if	(HAL_GAI > 0)
		"\tmode /gai idx attn=0~11db width=9~11]\r\n"
		#endif
		#if	(HAL_XXO > 0)
		"\tmode /act idx TBC\r\n"
		#endif
	"GMAP\tsense /uri idx Tsns Tlog [s1 [s2 [s3]]]\r\n"
	"GMAP\trule [ver] [val] IF /uri [idx] {cond} [AND/OR /uri [idx] {cond] THEN {actuation} ALSO {actuation}\r\n"
	strCRLF
};

// #################################### Public variables ##########################################

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
int	xCommandBuffer(report_t * psR, u8_t cCmd, bool bEcho) {
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
			if (cmdFlag.idx) cmdFlag.his = 1;
		} else if (cCmd == CHR_B) {						// Cursor DOWN
			cmdFlag.idx = xUBufStringPrv(psHB, cmdBuf, sizeof(cmdBuf));
			if (cmdFlag.idx) cmdFlag.his = 1;
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
		if (cCmd == CHR_CR || cCmd == CHR_LF) {
			if (cmdFlag.idx) {							// CR and something in buffer?
				cmdBuf[cmdFlag.idx] = 0;				// terminate command
				wprintfx(psR, strCRLF);
				iRV = xRulesProcessText((char *)cmdBuf);// then execute
				if (cmdFlag.his == 0) {					// if new/modified command
					vUBufStringAdd(psHB, cmdBuf, cmdFlag.idx); // save into buffer
				}
			}
			cmdFlag.u16 = 0;

		} else if (cCmd == CHR_BS || cCmd == CHR_DEL) {	// BS (macOS screen DEL) to remove previous character
			if (cmdFlag.idx) {							// yes,
				--cmdFlag.idx;							// step 1 slot back
				if (cmdFlag.idx == 0) {
					cmdFlag.u16 = 0;	// buffer empty, reset to default (non cli/history) mode
				}
			}

		} else if (isprint(cCmd) && (cmdFlag.idx < (sizeof(cmdBuf) - 1))) {	// printable and space in buffer
			cmdBuf[cmdFlag.idx++] = cCmd;				// store character & step index

		} else if (cCmd != CHR_LF) {
			xCommandReport(psR, cCmd);
		}
		cmdFlag.his = 0;
	}
	if (bEcho)
		wprintfx(psR, "\r\033[0K");						// if requested clear line
	if (cmdFlag.idx) {									// anything in buffer?
		cmdFlag.cli = 1;								// ensure flag is set
		if (bEcho) {
			wprintfx(psR, "%.*s \b", cmdFlag.idx, cmdBuf);	// optional refresh whole line
		}
	}
	return iRV;
}

// ################################# command string/character support ##############################

static void vCommandInterpret(command_t * psC) {
	int iRV = erSUCCESS;
	u8_t cCmd = *psC->pCmd++;
	if (cmdFlag.cli) {
		xCommandBuffer(&psC->sRprt, cCmd, psC->sRprt.fEcho);
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
		case CHR_SUB: 
			setSYSFLAGS(sfKEY_EOF);
			break;

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
			#if (buildPLTFRM == HW_EM1P2)
			if (cCmd < 3) {
				m90e26LoadNVSConfig(0, cCmd);
				m90e26LoadNVSConfig(1, cCmd);
			} else
			#elif (buildPLTFRM == HW_EM3P2)
			if (cCmd < 3) {
				m90e36Report();
				m90e36LoadNVSConfig(0, cCmd);
				m90e36LoadNVSConfig(1, cCmd);
				m90e36Report();
			} else
			#elif (buildPLTFRM==HW_AC00 || buildPLTFRM==HW_AC01 || buildPLTFRM==HW_DK41 || buildPLTFRM==HW_KC868A4 || buildPLTFRM==HW_KC868A6 || buildPLTFRM==HW_SP1PM || buildPLTFRM==HW_SP2PM)
			if (cCmd < HAL_XDO) {
				vActuatorLoad(cCmd, 5, 0, 500, 0, 500);
				#if	(buildPLTFRM == HW_AC00 || buildPLTFRM == HW_AC01)
				vActuatorLoad(cCmd + 8, 1, 0, 6000, 0, 0);
				#endif
				default:
					break;
				}
			} else
			#endif
			{	
				iRV = erOUT_OF_RANGE; 
			}
			break;

		#if	(HAL_XXO > 0)
		case CHR_A: vTaskActuatorReport(&psC->sRprt); break;
		#endif

		case CHR_B: {
			#define	blobBUFFER_SIZE			1024
			u8_t * pBuffer = malloc(blobBUFFER_SIZE);
			size_t	SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_PART, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_WIFI, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_VARS, pBuffer, &SizeBlob);
			#if	(buildPLTFRM == HW_EM1P2)
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, m90e26STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			#if	(buildPLTFRM == HW_SP2PM)
			SizeBlob = blobBUFFER_SIZE;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, ade7953STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			free(pBuffer);
			break;
		}

		#if	(halUSE_LITTLEFS == 1)
		case CHR_C:
			psC->sRprt.sFM.u32Val = makeMASK08x24(0,1,0,0,0,1,1,0,0x0);
			halSTORAGE_InfoFS(&psC->sRprt, "");
			break;
		#endif

		case CHR_D:
			psC->sRprt.sFM.aNL = 1;
			#if (HAL_GAI > 0)
			halGAI_Report(&psC->sRprt);
			#endif
			#if (HAL_GAO > 0)
			halGAO_Report(&psC->sRprt);
			#endif
			#if (HAL_GDI > 0)
			halGDI_Report(&psC->sRprt);
			#endif
			#if (HAL_ADE7953 > 0)
			ade7953Report(&psC->sRprt);
			#endif
			#if	(HAL_DS1307 > 0)
			ds1307Report(&psC->sRprt, strNUL);
			#endif
			#if	(HAL_LIS2HH12 > 0)
			lis2hh12ReportAll(&psC->sRprt);
			#endif
			#if	(HAL_LTR329ALS > 0)
			ltr329alsReportAll(&psC->sRprt);
			#endif
			#if	(HAL_M90E26 > 0)
			m90e26Report(&psC->sRprt);
			#endif
			#if	(HAL_MCP342X > 0)
			mcp342xReportAll(&psC->sRprt);
			#endif
			#if	(HAL_MPL3115 > 0)
			mpl3115ReportAll(&psC->sRprt);
			#endif
			#if	(HAL_ONEWIRE > 0)
			OWP_Report(&psC->sRprt);
			#endif
			#if (HAL_PCA9555 > 0)
			pca9555Report(&psC->sRprt);
			#endif
			#if (HAL_PCF8574 > 0)
			pcf8574Report(&psC->sRprt);
			#endif
			#if (HAL_PYCOPROC > 0)
			pycoprocReportAll(&psC->sRprt);
			#endif
			#if	(HAL_SI70XX > 0)
			si70xxReportAll(&psC->sRprt);
			#endif
			#if	(HAL_SSD1306 > 0)
			ssd1306Report(&psC->sRprt);
			#endif
			halWL_TimeoutReport(&psC->sRprt);
			vUBufReport(psHB);
			break;
		#endif						// (configPRODUCTION == 0)

		// ############################ Normal (non-dangerous) options
		case CHR_F:
			psC->sRprt.fForce = 1; 
			halVARS_ReportFlags(&psC->sRprt); 
			psC->sRprt.fForce = 0;
			break;

		case CHR_H: printfx(HelpMessage); break;

		case CHR_I:
			#if	(appUSE_IDENT > 0)
				vID_Report(&psC->sRprt);
			#else
			printfx("No identity support\r\n");
			#endif
			break;

		case CHR_L:
			halVARS_ReportGLinfo(&psC->sRprt); 
			halVARS_ReportTZinfo(&psC->sRprt);
			break;

		case CHR_M:
			psC->sRprt.sFM = (fm_t) makeMASK09x23(0,0,1,1,0,0,0,0,1,0x00FC0F);
			halMEM_ReportHistory(&psC->sRprt);
			halMEM_Report(&psC->sRprt);
//			xRtosReportMemory(&psC->sRprt);
			break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_N: xNetReportStats(&psC->sRprt); break;
		#endif

		case CHR_O: vOptionsShow(&psC->sRprt); break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_P: halFOTA_ReportPartitions(); break ;
		#endif

		case CHR_R: vRulesDecode(); break;

		case CHR_S:
			psC->sRprt.sFM.u32Val = makeMASK12x20(1,1,1,1,1,1,1,1,1,1,1,1, 0x000FFFFF);
			xTaskSensorsReport(&psC->sRprt);
			break;

		#if	(configPRODUCTION == 0)
		case CHR_T: vSysTimerShow(&psC->sRprt, 0xFFFFFFFF); break;
		#endif

		case CHR_U:
			psC->sRprt.sFM.u32Val = makeMASK09x23(1,1,1,1,1,1,1,0,1, 0x007FFFFF);
			xRtosReportTasks(&psC->sRprt);
			break;

		case CHR_V:
			halMCU_Report(&psC->sRprt);
			halWL_ReportLx(&psC->sRprt);
			vSyslogReport(&psC->sRprt);
			#if (includeHTTP_TASK > 0)
				vHttpReport(&psC->sRprt);
			#endif
			#if (includeTNET_TASK > 0)
				vTnetReport(&psC->sRprt);
			#endif
			#if (HAL_MB_SEN > 0 || HAL_MB_ACT > 0)
				xEpMBC_ClientReport(&psC->sRprt);
			#endif

			#if (buildAEP > 0)
				// flags for RX/TX (x32MMA) stats reporting
				psC->sRprt.sFM.u32Val = makeMASK09x23(1,0,1,1,1,1,1,1,1,0x000000);
				vAEP_Report(&psC->sRprt);
			#endif
			halVARS_ReportApp(&psC->sRprt);
			break;

		case CHR_W:
			halWL_Report(&psC->sRprt); 
			break;
		default: 
			xCommandBuffer(&psC->sRprt, cCmd, psC->sRprt.fEcho);
		}
	}
	if (iRV < erSUCCESS)
		xSyslogError(__FUNCTION__, iRV);
}

/**
 * @brief	process a command string and call the [optional handler] to process the buffered output
 * @return	number of characters passed to output
 */
int xCommandProcess(command_t * psC) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psC));
	int iRV = 0;
	if (buildSTDOUT_LEVEL > 0)
		xStdioBufLock(portMAX_DELAY);					// buffering enabled, lock
	if (psC->sRprt.fFlags)
		halVARS_ReportFlags(&psC->sRprt);				// handle flag changes
	while (psC->pCmd && *psC->pCmd) {
		vCommandInterpret(psC);							// process it..
		++iRV;
	}
	if (iRV > 1)	// if >1 character supplied, add CR to route through RULES engine
		xCommandBuffer(&psC->sRprt, CHR_CR, psC->sRprt.fEcho);
	halVARS_CheckChanges();								// check if VARS changed, write to NVS
	if (psC->sRprt.fFlags)
		halVARS_ReportFlags(&psC->sRprt);				// handle flag changes
	if (psC->Hdlr)
		iRV = psC->Hdlr(psC->pVoid);					// Empty buffer if required
	if (buildSTDOUT_LEVEL > 0)
		xStdioBufUnLock();								// buffering enabled, unlock
	return iRV;
}

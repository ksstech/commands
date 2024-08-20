// commands.c - Copyright (c) 2017-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"
#include "hal_device_includes.h"
#if (halUSE_I2C > 0)
	#include "hal_i2c_common.h"
#endif
#include "hal_flash.h"
#include "hal_gpio.h"
#include "hal_mcu.h"				// halMCU_Report()
#include "hal_memory.h"
#include "hal_network.h"
#include "hal_options.h"
#include "hal_stdio.h"
#include "hal_usart.h"

#include "actuators.h"
#include "commands.h"
#if (appUSE_IDENT > 0)
	#include "identity.h"
#endif
#include "task_aep.h"
#include "task_sensors.h"

#include "printfx.h"
#include "rules.h"					// xRulesProcessText
#include "statistics.h"
#include "syslog.h"
#include "systiming.h"

#include "x_builddefs.h"
#include "x_errors_events.h"
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
	"ESP32 Specific:" strNL
	"\tc-A/B/C Boot OTA #1/2/3 FW as STA" strNL
	"\tc-E delete 'syslog.txt'" strNL
	"\tc-P switch Platform & reboot" strNL
	"\tc-Q Toggle QOS 0->1->2->0" strNL
	"\tc-R Revert to previous FW" strNL
	#if (configPRODUCTION == 0)
	"\tc-T Immediate restart" strNL
	"\tc-U Generate 'Invalid memory access' crash" strNL
	#endif
	"\tc-V Reboot current FW as [AP]STA (delete VARS blob)" strNL
	"\tc-W Reboot current FW as APSTA (delete WIFI & VARS blobs)" strNL
	#endif

	"General:" strNL
	#if	(configPRODUCTION == 0) && (HAL_XXO > 0)
	"ACT\t(0-x) Trigger selected actuator" strNL
	#endif
	#if	(HAL_XXO > 0)
	"ACT\t(A)ctuators Report" strNL
	#endif
	#if	(configPRODUCTION == 0)
	"\t(B)lob report" strNL
	#if	(halUSE_LITTLEFS == 1)
	"\t(C)ontent of LFS" strNL
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
	"]" strNL
	#endif

	"\t(F)lags Status" strNL
	"\t(H)elp screen display" strNL
	#if	(appUSE_IDENT > 0)
	"\t(I)dent table" strNL
	#endif
	"\t(L)ocation info" strNL
	"\t(M)emory info" strNL
	"\t(N)etwork (IP4) info" strNL
	"\t(O)ptions display" strNL
	#if	(configPRODUCTION == 0)
	"\t(P)artitions report" strNL
	#endif
	"\t(R)ules display" strNL
	"\t(S)ensors statistics" strNL
	#if	(configPRODUCTION == 0)
	"\t(T)imer/Scatter Info" strNL
	#endif
	"\t(U)tilization (task) statistics" strNL
	"\t(V)erbose system info" strNL
	"\t(W)ifi Stats" strNL

	"Extended commands:" strNL
	"\treboot | register | upgrade | show W0 [... [W23]]" strNL
	#if	(HAL_XXO > 0)
	"ACT\tdispense ch# fld# Rpt tFI tON tFO tOFF Amt" strNL
	"ACT\tload|update ch# Rpt tFI tON tFO tOFF" strNL
	"ACT\tadjust ch# stage# Adj" strNL
	"ACT\tque|seq ch# S0 [... S23]]" strNL
	#endif
	"GMAP\tioset option para1 para2" strNL
	"GMAP\tioset 141(nwmo) {0->3} off/sta/ap/sta+ap" strNL
	"GMAP\tioset 142(wifi) idx (-1 -> 3) ssid(u8 x23) pswd(u8 x23)" strNL
	"GMAP\tioset 143(mqtt) w.x.y.z port" strNL
	#if	(configPRODUCTION == 0)
		"GMAP\tioset 144(peek) address size" strNL
		"GMAP\tioset 145(poke) address size" strNL
	#endif

	"GMAP\tmode /uri para1 [para2 .. [para6]]" strNL
		#if (HAL_ADE7953 > 0)
		"\tmode /ade7953 idx ???" strNL
		#endif
		#if	(HAL_DS18X20 > 0)
		"\tmode /ds18x20 idx lo=-128~127 hi=-128~127 res=9~12 wr=0/1" strNL
		#endif
		#if	(HAL_LIS2HH12 > 0)
		"\tmode /lis2hh12 0 ths(0-127) dur(0-255)" strNL
		"\tmode /lis2hh12 1 hr(0/1) odr(0-7) bdu(0/1) ?en(0->7)" strNL
		"\tmode /lis2hh12 3 CTRL1  IG_CFG1  " strNL
		"\tmode /lis2hh12 4 bw(0->3) fs(0->3) bw(0/1) Aincr(0/1)" strNL
		#endif
		#if	(HAL_LTR329ALS > 0)
		"\tmode /ltr329als idx gain=0~3/6/7 time=0~7 rate=0~7" strNL
		#endif
		#if	(HAL_M90E26 > 0)
		"\tmode /m90e26 idx 1=gainL val=1/4/8/16/24" strNL
		#if	(m90e26NEUTRAL > 0)
		"\t\t2=gainN val=1/2/4" strNL
		#endif
		"\t\t4=reCalib" strNL
		"\t\t5=Calc CurOfst" strNL
		"\t\t6=Calc PwrOfst" strNL
		"\t\t7=Save pos=0~" toSTR(m90e26CALIB_NUM-1) " 'Calibration Data'" strNL
		"\t\t8=Delete 'ALL Calibration data'" strNL
		"\t\t9=WriteReg reg=" toSTR(SOFTRESET) "~" toSTR(CRC_2) " val=0~0xFFFF" strNL
		#endif
		#if (configPRODUCTION == 0) && (HAL_MB_ACT > 0 || HAL_MB_SEN > 0)
		"\tcmd /mb TBC" strNL
		#endif
		#if (HAL_MB_ACT > 0)
		"\tcmd /mb/act " strNL
		"\tmode /mb/act idx TBC" strNL
		#endif
		#if (HAL_MB_SEN > 0)
		"\tcmd /mb/sen  " strNL
		"\tmode /mb/sen idx TBC" strNL
		#endif
		#if	(HAL_MCP342X > 0)
		"\tmode /mcp342x idx TBC" strNL
		#endif
		#if	(HAL_MPL3115 > 0)
		"\tmode /mpl3115 idx TBC" strNL
		#endif
		#if	(HAL_PYCOPROC > 0)
		"\tmode /pycoproc idx TBC" strNL
		#endif
		#if	(HAL_SI70XX > 0)
		"\tmode /si70xx idx TBC" strNL
		#endif
		#if (HAL_GDI > 0)
		"\tmode /gdi idx inv=0~1 type=0~5 dly=0~255" strNL
		#endif
		#if	(HAL_GAI > 0)
		"\tmode /gai idx attn=0~11db width=9~11]" strNL
		#endif
		#if	(HAL_XXO > 0)
		"\tmode /act idx TBC" strNL
		#endif
	"GMAP\tsense /uri idx Tsns Tlog [s1 [s2 [s3]]]" strNL
	"GMAP\trule [ver] [val] IF /uri [idx] {cond} [AND/OR /uri [idx] {cond] THEN {actuation} ALSO {actuation}" strNL
	strNL
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

ubuf_t * psHB = NULL;

// ############################### UART/TNET/HTTP Command interpreter ##############################

void xCommandReport(report_t * psR, int cCmd) {
	wprintfx(psR, "E=%d L=%d H=%d I=%d cCmd=%d" strNL, cmdFlag.esc, cmdFlag.lsb, cmdFlag.his, cmdFlag.idx, cCmd);
}

/**
 * @brief
 * @param
 * @param
 * @param
 * @return
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
				wprintfx(psR, strNL);
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
		case CHR_ETX: halFlashSetBootNumber(cCmd, fotaBOOT_REBOOT); break;		// c-C
		case CHR_ENQ: unlink("syslog.txt"); break;								// c-E
		case CHR_DC2: halFlashSetBootNumber(PrvPart, fotaBOOT_REBOOT); break;	// c-R
		case CHR_DC4: esp_restart(); break;				// c-T Immediate restart
		case CHR_NAK: *((char *)0xFFFFFFFF)=1; break;	// c-U Illegal memory write crash
		case CHR_SYN:									// c-V Erase VARS blob then reboot
			halFlashSetBootNumber(CurPart, fotaERASE_VARS|fotaBOOT_REBOOT);
			break;
		case CHR_ETB:				// c-W Erase VARS,WIFI M90E26/ADE7953 blobs then reboot
			halFlashSetBootNumber(CurPart, fotaERASE_WIFI|fotaERASE_VARS|fotaERASE_DEVNVS|fotaBOOT_REBOOT);
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
		{	cCmd -= CHR_0;
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
			if (cCmd < HAL_XXO) {
				u8_t Type = xActuatorGetType(cCmd);
				switch(Type) {
				#if (HAL_XDO > 0)
					case actTYPE_DIG:
					#if	(buildPLTFRM == HW_AC00 || buildPLTFRM == HW_AC01)
						vActuatorLoad(cCmd, 5, 0, 500, 0, 500);				// LED 0~7
						vActuatorLoad(cCmd + 8, 1, 0, 6000, 0, 0);			// Relay 0~7
					#elif (buildPLTFRM == HW_DK41)
						vActuatorLoad(cCmd, 5, 0, 500, 0, 500);				// LED 0~2
					#elif (buildPLTFRM == HW_KC868A4 || buildPLTFRM == HW_KC868A6)
						vActuatorLoad(cCmd, 1, 0, 10, 0, 0);				// Relay 0~5
					#elif (buildPLTFRM == HW_SP1PM || buildPLTFRM == HW_SP2PM)
						vActuatorLoad(cCmd, 3, 0, 1000, 0, 1000);			// Relay 0~1
					#endif
					break;
				#endif
				#if	(buildPLTFRM == HW_KC868A4 || buildPLTFRM == HW_KC868A6)
					case actTYPE_ANA:
						vActuatorLoad(cCmd, 5, 250, 250, 250, 250);			// DAC 0~1
						break;
				#endif
				default:
					break;
				}
			} else
			#endif
			{	
				iRV = erOUT_OF_RANGE; 
			}
		}	break;

		#if	(HAL_XXO > 0)
		case CHR_A: xTaskActuatorReport(&psC->sRprt); break;
		#endif

		case CHR_B: {					// List blobs with contents
			#define	blobBUFFER_SIZE			1024
			u8_t * pBuffer = malloc(blobBUFFER_SIZE);
			size_t	SizeBlob = blobBUFFER_SIZE;
			halFlashReportBlob(&psC->sRprt, halFLASH_STORE, halFLASH_KEY_PART, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halFlashReportBlob(&psC->sRprt, halFLASH_STORE, halFLASH_KEY_WIFI, pBuffer, &SizeBlob);
			SizeBlob = blobBUFFER_SIZE;
			halFlashReportBlob(&psC->sRprt, halFLASH_STORE, halFLASH_KEY_VARS, pBuffer, &SizeBlob);
			#if	(buildPLTFRM == HW_EM1P2)
				SizeBlob = blobBUFFER_SIZE;
				halFlashReportBlob(&psC->sRprt, halFLASH_STORE, m90e26STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			#if	(buildPLTFRM == HW_SP2PM)
				SizeBlob = blobBUFFER_SIZE;
				halFlashReportBlob(&psC->sRprt, halFLASH_STORE, ade7953STORAGE_KEY, pBuffer, &SizeBlob);
			#endif
			free(pBuffer);
		}	break;

		#if	(halUSE_LITTLEFS == 1)
		case CHR_C: {
			psC->sRprt.sFM.u32Val = (ioB2GET(ioFSlev) == 3) ? makeMASK08x24(0,1,1,1,1,1,0,0,0) :
									(ioB2GET(ioFSlev) == 2) ? makeMASK08x24(0,1,1,1,1,0,0,0,0) :
									(ioB2GET(ioFSlev) == 1) ? makeMASK08x24(0,1,1,1,0,0,0,0,0) :
															  makeMASK08x24(0,1,1,0,0,0,0,0,0) ;
			halFlashInfoFS(&psC->sRprt, "");
		}	break;
		#endif

		case CHR_D:
		{	psC->sRprt.sFM.aNL = 1;
			#if (HAL_GAI > 0)
			halGAI_Report(&psC->sRprt);
			#endif
			#if (HAL_GAO > 0)
			halGAO_Report(&psC->sRprt);
			#endif
			#if (HAL_GDI > 0)
			halGDI_Report(&psC->sRprt);
			#endif
			#if (HAL_GDO > 0)
			halGDO_Report(&psC->sRprt);
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
			vUBufReport(&psC->sRprt, psHB);
		}	break;
		#endif						// (configPRODUCTION == 0)

		// ############################ Normal (non-dangerous) options
		case CHR_F: {
			psC->sRprt.fForce = 1; 
			halVARS_ReportFlags(&psC->sRprt); 
			psC->sRprt.fForce = 0;
		}	break;

		case CHR_H: wprintfx(&psC->sRprt, "%s", HelpMessage); break;

		case CHR_I: {
			#if	(appUSE_IDENT > 0)
				vID_Report(&psC->sRprt);
			#else
				wprintfx(&psC->sRprt, "No identity support" strNL);
			#endif
		}	break;

		case CHR_L: halVARS_ReportGLinfo(&psC->sRprt); break;

		case CHR_M: {
			psC->sRprt.sFM.u32Val = makeMASK12x20(0,1,0,1,1,1,1,1,1,0,1,0,0x00FC0F);
			halMEM_HistoryReport(&psC->sRprt);
			halMEM_SystemReport(&psC->sRprt);
//			xRtosReportMemory(&psC->sRprt);
		}	break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_N: xNetReportStats(&psC->sRprt); break;
		#endif

		case CHR_O: vOptionsShow(&psC->sRprt); break;

		#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_P: halFlashReportPartitions(&psC->sRprt); break;
		#endif

		case CHR_R: vRulesDecode(&psC->sRprt); break;

		case CHR_S: {
			psC->sRprt.sFM.u32Val = makeMASK12x20(1,1,1,1,1,1,1,1,1,1,1,1, 0x000FFFFF);
			xTaskSensorsReport(&psC->sRprt);
		}	break;

		#if	(configPRODUCTION == 0)
		case CHR_T: vSysTimerShow(&psC->sRprt, 0xFFFFFFFF); break;
		#endif

		case CHR_U: {
			psC->sRprt.sFM.u32Val = makeMASK09x23(1,1,1,1,1,1,1,0,1, 0x007FFFFF);
			xRtosReportTasks(&psC->sRprt);
		
		}	break;

		case CHR_V: {
			halMCU_Report(&psC->sRprt);
			halWL_ReportLx(&psC->sRprt);
			vSyslogReport(&psC->sRprt);
			#if (includeTNET_TASK > 0)
				vTnetReport(&psC->sRprt);
			#endif
			#if (HAL_MB_SEN > 0 || HAL_MB_ACT > 0)
				xEpMBC_ClientReport(&psC->sRprt);
			#endif

			#if (buildAEP > 0)
				psC->sRprt.sFM.u32Val = makeMASK09x23(1,0,1,1,1,1,1,1,1,0x000000);
				xAEP_Report(&psC->sRprt);
			#endif
			psC->sRprt.sFM.aNL = 1;
			halVARS_ReportApp(&psC->sRprt);
		}	break;

		case CHR_W: halWL_Report(&psC->sRprt); break;

		#if (halFLASH_FIX_MD5 == 1)
		case CHR_X: halFlashReportMD5(); break;
		case CHR_Y: halFlashRemoveMD5(); break;
		case CHR_Z: halFlashRestoreMD5(); break;
		#endif

		default: xCommandBuffer(&psC->sRprt, cCmd, psC->sRprt.fEcho);
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
	IF_myASSERT(debugPARAM, halMemorySRAM(psC));
	int iRV = 0;
	if (psHB == NULL) { 			// init history buffer, variable size, blocks of 128 bytes
		psHB = psUBufCreate(NULL, NULL, (ioB4GET(ioCLIbuf)+1) << 7, 0);
		psHB->f_history = 1;
	}
	#if (configCONSOLE_UART > (-1))
		xStdioBufLock(portMAX_DELAY);					// buffering enabled, lock
	#endif
	if (psC->sRprt.fFlags)
		halVARS_ReportFlags(&psC->sRprt);				// handle flag changes
	while (psC->pCmd && *psC->pCmd) {
		vCommandInterpret(psC);							// process it..
		++iRV;
	}
	if (iRV > 1)					// if >1 character supplied, add CR to route through RULES engine
		xCommandBuffer(&psC->sRprt, CHR_CR, psC->sRprt.fEcho);
	halVARS_CheckChanges();								// check if VARS changed, write to NVS
	if (psC->sRprt.fFlags)
		halVARS_ReportFlags(&psC->sRprt);				// handle flag changes
	#if (configCONSOLE_UART > (-1))
		xStdioBufUnLock();								// buffering enabled, unlock
	#endif
	return iRV;
}

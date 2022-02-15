/*
 * commands.c - command interpreter
 */

#include	"hal_variables.h"
#include	"commands.h"
#include	"hbuf.h"

#include	"FreeRTOS_Support.h"						// freertos statistics complex_vars struct_unions x_time definitions stdint time
#include	"actuators.h"

#if (SW_AEP == 1)
	#include	"task_sitewhere.h"
#elif	(SW_AEP == 2)
	#include	"task_thingsboard.h"
#endif

#if (configUSE_IDENT == 1)
	#include	"ident1.h"
#elif (configUSE_IDENT == 2)
	#include	"ident2.h"
#endif

#include	"options.h"
#include 	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"

#include	"x_http_server.h"
#include	"x_string_general.h"						// xstrncmp()
#include	"x_string_to_values.h"

#include	"x_errors_events.h"
#include	"x_builddefs.h"
#include	"x_telnet_server.h"

#include	"hal_usart.h"
#include 	"hal_mcu.h"									// halMCU_Report()
#include	"hal_fota.h"
#include	"hal_storage.h"

#include	"MQTTClient.h"								// QOSx levels

#if (configUSE_RULES > 0)
	#include	"rules_decode.h"
	#include	"rules_parse_text.h"
#endif

#if (halHAS_LIS2HH12 > 0)
	#include	"lis2hh12.h"
#endif

#if (halHAS_LTR329ALS > 0)
	#include	"ltr329als.h"
#endif

#if (halHAS_M90E26 > 0)
	#include	"m90e26.h"
#endif

#if (halHAS_MCP342X > 0)
	#include	"mcp342x.h"
#endif

#if (halHAS_MPL3115 > 0)
	#include	"mpl3115.h"
#endif

#if (halHAS_ONEWIRE > 0)
	#include	"onewire_platform.h"
	#if	(halHAS_DS18X20 > 0)
	#include	"ds18x20.h"
	#endif
#endif

#if (halHAS_PCA9555 > 0)
	#include	"pca9555.h"
#endif

#if (halHAS_PYCOPROC > 0)
	#include	"pycoproc.h"
#endif

#if (halHAS_SI70XX > 0)
	#include	"si70xx.h"
#endif

#if (halHAS_SSD1306 > 0)
	#include	"ssd1306.h"
#endif

#include	<string.h>
#include	<stdbool.h>

#define	debugFLAG					0xC002

#define	debugCMND					(debugFLAG & 0x0001)
#define	debugLEVEL					(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Macros ################################################

#define	bufferMEMDUMP_SIZE			(1 * KILO)

// ############################### Forward function declarations ##################################


// #################################### Public variables ##########################################


// ##################################### Local variables ##########################################

static const char HelpMessage[] = {
	#ifdef ESP_PLATFORM
	"ESP32 Specific:\n"
	"\tc-A Boot OTA #1 FW as STA\n"
	#if (fotaMAX_OTA_PARTITIONS > 2)
	"\tc-B Boot OTA #2 FW as STA\n"
	#endif
	#if (fotaMAX_OTA_PARTITIONS > 3)
	"\tc-C Boot OTA #3 FW as STA\n"
	#endif
	"\tc-P switch Platform & reboot\n"
	"\tc-Q Toggle QOS 0->1->2->0\n"
	"\tc-R Revert to previous FW\n"
	"\tc-V Reboot current FW as APSTA (delete WIFI & VARS blobs)\n"
	"\tc-W Reboot current FW as [AP]STA (delete VARS blob)\n"
	#if (!defined(NDEBUG) || defined(DEBUG))
	"\tc-T Generate WatchDog timeout\n"
	"\tc-U generate Invalid memory access crash\n"
	#endif
	#endif

	"General:\n"
	#if	(configPRODUCTION == 0) && (halXXX_XXX_OUT > 0)
	"ACT\t(0-x) Trigger selected actuator\n"
	#endif
	#if	(halXXX_XXX_OUT > 0)
	"ACT\t(A)ctuators Report\n"
	#endif
	#if	(configPRODUCTION == 0)
	"\t(B)lob report\n"
	#if	(halUSE_LITTLEFS == 1)
	"\t(C)ontent of LFS\n"
	#endif
	"\t(D)iagnostics\n"
	#if (halSOC_DIG_IN > 0)
	"\t    DigIn Pins\n"
	#endif
	#if	(halHAS_DS18X20 > 0)
	"1W\t    DS18X20 device info\n"
	#endif
	#if (halHAS_LIS2HH12 > 0)
	"lis2hh12    lis2hh12Report\n"
	#endif
	#if	(halHAS_LTR329ALS > 0)
	"ltr329als   ltr329alsReport\n"
	#endif
	#if	(halHAS_M90E26 > 0)
	"M90E\t    m90e26Report\n"
	#endif
	#if	(halHAS_MCP342X > 0)
	"MCP342x\t    mcp342xReport\n"
	#endif
	#if	(halHAS_MPL3115 > 0)
	"mpl3115     mpl3115Report\n"
	#endif
	#if	(halHAS_ONEWIRE > 0)
	"1W\t    Onewire info\n"
	#endif
	#if	(halHAS_SI70XX > 0)
	"si70xx      si70xxReport\n"
	#endif
	#if	(halHAS_SSD1306 > 0)
	"SSD1306\t    ssd1306Report\n"
	#endif
	#endif

	"\t(F)lags Status\n"
	"\t(H)elp screen display\n"
	#if	(configUSE_IDENT > 0)
	"\t(I)dent table\n"
	#endif
	"\t(L)ocation info\n"
	"\t(M)emory info\n"
	"\t(N)etwork (IP4) info\n"
	"\t(O)ptions display\n"
	#if	(configPRODUCTION == 0)
	"\t(P)artitions report\n"
	#endif
	"\t(R)ules display\n"
	"\t(S)ensors statistics\n"
	#if	(configPRODUCTION == 0)
	"\t(T)imer/Scatter Info\n"
	#endif
	"\t(U)tilization (task) statistics\n"
	"\t(V)erbose system info\n"
	"\t(W)ifi Stats\n"

	"Extended commands:\n"
	#if (halHAS_LIS2HH12 > 0)
	"mode /lis2hh12 idx ths(0-127) dur(0-255) odr(0-7) hr(0/1)\n"
	#endif
	#if	(halXXX_XXX_OUT > 0)
	"ACT\tdispense ch# fld# Rpt tFI tON tFO tOFF Amt\n"
	"ACT\tload|update ch# Rpt tFI tON tFO tOFF\n"
	"ACT\tadjust ch# stage# Adj\n"
	"ACT\tque|seq ch# S0 [... S23]]\n"
	#endif
	"GMAP\tioset 134 idx (-1 -> 3) ssid pswd\n"
	"GMAP\tioset option para1 para2\n"
	"GMAP\tmode /uri para1 [para2 .. [para6]]\n"
	"GMAP\trule [ver] [val] IF /uri [idx] {cond} [AND/OR /uri [idx] {cond] THEN {actuation} ALSO {actuation}\n"
	"GMAP\tsense /uri idx Tsns Tlog [s1 [s2 [s3]]]\n"
	"\tregister\n"
	"\trestart\n"
	"\tshow W0 [... [W23]]\n"
	"\tupgrade\n"
	"\n"
} ;

static hbuf_t sCLI = { 0 };
static uint8_t Lbuf[127]= { 0 };
static uint8_t Lidx = 0;

// ################################ Forward function declarations ##################################

void halWL_Report(void) ;
void halWL_ReportLx(void) ;
void vTaskSensorsReport(void) ;
void vControlReportTimeout(void) ;

// ############################### UART/TNET/HTTP Command interpreter ##############################

int	xCommandBuffer(int cCmd, bool bEcho) {
	int iRV = erSUCCESS;
	hbuf_t * psHB = &sCLI;
	if (allSYSFLAGS(sfESCAPE|sfLSBRACKET)) {
		Lidx = 0;
		if (psHB->iCur == psHB->iFree) {				// history empty?
			// Do nothing
		} else if (cCmd == CHR_A) {						// Cursor UP
			Lidx = vHBufPrvCmd(psHB, Lbuf, sizeof(Lbuf));
			if (Lidx)
				setSYSFLAGS(sfHISTORY);
		} else if (cCmd == CHR_B) {						// Cursor DOWN
			Lidx = vHBufNxtCmd(psHB, Lbuf, sizeof(Lbuf));
			if (Lidx)
				setSYSFLAGS(sfHISTORY);
		} else {										// unknown command....
			// Do nothing
		}
	} else {
		if (cCmd == '\r' && Lidx) {				// CR and something in buffer?
			if (bEcho) {								// yes, ....
				printfx(" -> ");
			}
			Lbuf[Lidx] = 0;								// terminate command
			iRV = xRulesProcessText((char *)Lbuf);		// then execute
			if (allSYSFLAGS(sfHISTORY) == 0) {			// if new/modified command
				vHBufAddCmd(psHB, Lbuf, Lidx);			// save into buffer
			}
			Lidx = 0;
		} else {
			if (cCmd == CHR_BS) {				// BS to remove last character
				if (Lidx) {
					--Lidx;
				}
			} else if (cCmd == CHR_ESC) {		// ESC to clear the buffer
				Lidx = 0;
			} else if (isprint(cCmd) &&			// printable character
				(Lidx < (sizeof(Lbuf) - 1))) {			// and space in buffer
				Lbuf[Lidx++] = cCmd;					// store character & step index
			}
		}
		clrSYSFLAGS(sfHISTORY);
	}
	clrSYSFLAGS(sfESCAPE|sfLSBRACKET);
	if (Lidx) {											// anything in buffer?
		setSYSFLAGS(sfCLI);								// ensure flag is set
		if (bEcho) {									// option refresh whole line
			printfx("\r%.*s \b", Lidx, Lbuf);
		}
	} else {
		clrSYSFLAGS(sfCLI);								// buffer empty, clear flag
		if (bEcho) {									// optional clear line
			printfx("\r\033[0K");
		}
	}
	return iRV;
}

void vCommandInterpret(int cCmd, bool bEcho) {
	halVARS_ReportFlags(0);
	if (cCmd == 0 || cCmd == EOF) {
		return;
	}
	if (allSYSFLAGS(sfCLI) || allSYSFLAGS(sfESCAPE|sfLSBRACKET)) {
		xCommandBuffer(cCmd, bEcho);
	} else {
		switch (cCmd) {
	#if defined(ESP_PLATFORM)
		case CHR_SOH: halFOTA_SetBootNumber(1, fotaBOOT_REBOOT); break;	// c-A
		case CHR_STX: halFOTA_SetBootNumber(2, fotaBOOT_REBOOT); break;	// c-B
		case CHR_ETX: halFOTA_SetBootNumber(3, fotaBOOT_REBOOT); break;	// c-C

		case CHR_DC2: halFOTA_RevertToPreviousFirmware(fotaBOOT_REBOOT); break;	// c-R
		case CHR_DC4: while(1); break;					// WatchDog timeout crash
		case CHR_NAK: *((char *) 0xFFFFFFFF) = 1; break;// Illegal memory write crash
		case CHR_SYN: halFOTA_SetBootNumber(halFOTA_GetBootNumber(),fotaERASE_WIFI|fotaERASE_VARS|fotaBOOT_REBOOT); break;	// c-V
		case CHR_ETB: halFOTA_SetBootNumber(halFOTA_GetBootNumber(), fotaERASE_VARS|fotaBOOT_REBOOT); break;	// c-W
	#endif

		case CHR_ESC:
			if (anySYSFLAGS(sfESCAPE|sfLSBRACKET)) {
				clrSYSFLAGS(sfESCAPE|sfLSBRACKET);
			} else {
				setSYSFLAGS(sfESCAPE);
			}
			break;

		// ########################### Unusual (possibly dangerous) options
	#if	(configPRODUCTION == 0)
		#if	(HW_VARIANT == HW_AC00) || (HW_VARIANT == HW_AC01)
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
		case CHR_4:
		case CHR_5:
		case CHR_6:
		case CHR_7:
			xActuatorLoad(cCmd - CHR_0 + 8, 1, 0, 6000, 0, 0) ;
			xActuatorLoad(cCmd - CHR_0, 6, 0, 500, 0, 500) ;
			break ;

		#elif (HW_VARIANT == HW_WROVERKIT)
		case CHR_0:
		case CHR_1:
		case CHR_2:
			if (cCmd - CHR_0 < halSOC_DIG_OUT)
				xActuatorLoad(cCmd - CHR_0, 5, 500, 500, 500, 500);
			else
				printfx("%c", CHR_BEL);
			break;

		#elif (HW_VARIANT == HW_DOITDEVKIT)
		case CHR_0:
		case CHR_1:
			if (cCmd - CHR_0 < halSOC_DIG_OUT)
				xActuatorLoad(cCmd - CHR_0, 5, 500, 500, 500, 500) ;
			else
				printfx("%c", CHR_BEL) ;
			break ;

		#elif (HW_VARIANT == HW_EM1P2)
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
			m90e26Report() ;
			m90e26LoadNVSConfig(0, cCmd - CHR_0) ;
			m90e26LoadNVSConfig(1, cCmd - CHR_0) ;
			m90e26Report() ;
			break ;
		#endif

		#if	(halXXX_XXX_OUT > 0)
		case CHR_A:	vTaskActuatorReport(); break;
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
			#if	(HW_VARIANT == HW_EM1P2)
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
			halGPIO_DIreport();
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
	#if	(configUSE_IDENT == 1)
		case CHR_I: vID1_ReportAll(); break;
	#elif (configUSE_IDENT == 2)
		case CHR_I: vID2_ReportAll(); break;
	#endif
//		case CHR_J: case CHR_K:
		case CHR_L: halVARS_ReportGeoloc(); break;
		case CHR_M: {
			flagmask_t sFM = { .u32Val = makeMASK11x21(1,0,0,1,1,1,1,1,1,1,1,0) };
			vRtosReportMemory(sFM, NULL, 0);
			break;
		}
		case CHR_N: xNetReportStats(); break;
		case CHR_O: vOptionsShow(); break;
	#if	defined(ESP_PLATFORM) && (configPRODUCTION == 0)
		case CHR_P: halFOTA_ReportPartitions(); break ;
	#endif
	#if	(configPRODUCTION == 0)
		case CHR_Q:
			sNVSvars.QoSLevel = (sNVSvars.QoSLevel == QOS0) ? QOS1 :
								(sNVSvars.QoSLevel == QOS1) ? QOS2 : QOS0;
			setSYSFLAGS(vfQOSLEVEL|sfRESTART);
			break;
	#endif
		case CHR_R: vRulesDecode(); break;
		case CHR_S: vTaskSensorsReport(); break;
	#if	(configPRODUCTION == 0)
		case CHR_T: vSysTimerShow(0xFFFFFFFF); break;
	#endif
		case CHR_U: {
			flagmask_t sFM = {.u32Val = makeMASK09x23(0,1,1,1,1,1,1,1,1,0x007FFFFF) };
			xRtosReportTasks(sFM, NULL, 0);
			break;
		}
		case CHR_V:
			halMCU_Report();
			halVARS_ReportFirmware();
			halWL_ReportLx();
			vSyslogReport();
			IF_EXEC_0(configCONSOLE_HTTP == 1, vHttpReport);
			IF_EXEC_0(configCONSOLE_TELNET == 1, vTelnetReport);
			#if	(SW_AEP == 1)
			#include "task_sitewhere.h"
			vSW_Report() ;
			#elif (SW_AEP == 2)
			#include "task_thingsboard.h"
			vTB_Report();
			#endif
			halVARS_ReportSystem();
			vControlReportTimeout();
			vHBufReport(&sCLI);
			break ;
		case CHR_W: halWL_Report(); break;
//		case CHR_X: case CHR_Y: case CHR_Z:
		case CHR_L_SQUARE:
			if (allSYSFLAGS(sfESCAPE) && allSYSFLAGS(sfLSBRACKET) == 0) {
				setSYSFLAGS(sfLSBRACKET);
			} else {
				clrSYSFLAGS(sfESCAPE|sfLSBRACKET);
			}
			break;
		default: xCommandBuffer(cCmd, bEcho);
		}
	}
	halVARS_ReportFlags(0);
}

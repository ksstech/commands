/*
 * commands.c - command interpreter
 */

#include	"hal_variables.h"
#include	"commands.h"

#include	"FreeRTOS_Support.h"						// freertos statistics complex_vars struct_unions x_time definitions stdint time
#include	"actuators.h"

#if		(SW_AEP == 1)
	#include	"task_sitewhere.h"
#elif	(SW_AEP == 2)
	#include	"task_thingsboard.h"
#endif

#if		(configUSE_IDENT == 1)
	#include	"ident1.h"
#elif	(configUSE_IDENT == 2)
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

#if		(configUSE_RULES > 0)
	#include	"rules_decode.h"
	#include	"rules_parse_text.h"
#endif

#if		(halHAS_SSD1306 > 0)
	#include	"ssd1306.h"
#endif

#if		(halHAS_M90E26 > 0)
	#include	"m90e26.h"
#endif

#if		(halHAS_MCP342X > 0)
	#include	"mcp342x.h"
#endif

#if		(halHAS_ONEWIRE > 0)
	#include	"onewire_platform.h"
#endif

#if		(halHAS_PCA9555 > 0)
	#include	"pca9555.h"
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
#define	cliBUF_SIZE					128

// ############################### Forward function declarations ##################################


// #################################### Public variables ##########################################


// ##################################### Local variables ##########################################

#if	(cliUSE_TABLE == 1)
cmnd_t sCLIlist[] = { {	"CMND",	CmndCMND }, } ;
#endif

static	char	CLIbuf[cliBUF_SIZE] = { 0 } ;
static	cli_t	sCLI ;

static const char	HelpMessage[] = {
#if		defined(ESP_PLATFORM)
	"ESP32 Specific\n"
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
	#if		(!defined(NDEBUG) || defined(DEBUG))
	"\tc-T Generate WatchDog timeout\n"
	"\tc-U generate Invalid memory access crash\n"
	#endif
	"\t(p)artitions report\n"
#endif

	"General\n"
	"\t(b)lob report\n"
	"\t(f)lags Status\n"
	"\t(h)elp screen display\n"
	"\t(l)ocation info\n"
	"\t(m)emory info\n"
	"\t(n)etwork (IP4) info\n"
	"\t(o)ptions display\n"
	"\t(r)ules display\n"
	"\t(s)ensors statistics\n"
	"\t(t)asks statistics\n"
	"\t(v)erbose system info\n"
	"\t(w)ifi Stats\n"
	"EXT\tz{ioset|sense|mode|rule} {text to decode}\n"

#if		(!defined(NDEBUG) || defined(DEBUG))
	"\tre(B)oot\n"
	"\t(T)imer/Scatter Info\n"
	"\t(U)pgrade Firmware\n"
#endif

#if		(halXXX_XXX_OUT > 0)
	"\t(a)ctuators status\n"
	#if	(!defined(NDEBUG) || defined(DEBUG))
	"\t(0-7) Trigger actuator channel 'x'\n"
	"\t(A)ctuators reload\n"
	#endif
#endif

#if		(configUSE_IDENT > 0)
	"\t(I)nit rules & ident\n"
	"\t(i)dent table\n"
#endif

#if		(halHAS_ONEWIRE > 0)
	"1-Wire\n"
	#if	(halHAS_DS248X > 0)
	"\t(C)DS248X flags level (0-1-2-3-0) increment\n"
	"\t(D)ebug 1W Channels\n"
	#endif
	"\t(O)newire info\n"
#endif

#if	(halHAS_M90E26 > 0)
	"M90E26\n"
	"\t(d)ebug M90E26[+SSD1306] config\n"
	#if		(!defined(NDEBUG) || defined(DEBUG))
	"\t(A)utomatic adjustment\n"
	"\t    Calibrate M90E26's\n"
	"\t(0-2) load predefined config 'x'\n"
	#endif
#endif
	"\n"
} ;

// ############################### UART/TNET/HTTP Command interpreter ##############################

#if	(cliUSE_TABLE == 1)
int	xCLImatch(cli_t * psCLI) {
	for (int Idx = 0; Idx < psCLI->u8LSize; ++Idx) {
		size_t Len = strnlen(psCLI->pasList[Idx].cmnd, SO_MEM(cmnd_t, cmnd)) ;
		if (xstrncmp(psCLI->pcParse, psCLI->pasList[Idx].cmnd, Len, 0) == 1) {
			psCLI->pcParse += Len ;
			return Idx ;
		}
	}
	return erFAILURE ;
}
#endif

void vCLIreset(cli_t * psCLI) {
	psCLI->pcStore	= psCLI->pcParse = psCLI->pcBeg = CLIbuf ;
	sCLI.u8BSize	= cliBUF_SIZE ;
#if	(cliUSE_TABLE == 1)
	sCLI.pasList	= sCLIlist ;
	sCLI.u8LSize	= NO_MEM(sCLIlist) ;
#endif
	psCLI->bMode	= 0 ;
	memset(psCLI->pcBeg, 0, psCLI->u8BSize) ;
}

void vCLIinit(void) { vCLIreset(&sCLI) ; }

char caBS[] = { CHR_BS, CHR_SPACE, CHR_BS, CHR_NUL } ;

int	xCommandBuffer(cli_t * psCLI, int cCmd) {
	int iRV = erSUCCESS ;
	if (cCmd == '\r' || cCmd == 0) {					// terminating char received
		*psCLI->pcStore	= 0 ;
		if (psCLI->bEcho) printfx("\n") ;
#if	(cliUSE_TABLE == 1)
		iRV = xCLImatch(psCLI) ;						// try to find matching command
		if (iRV > erFAILURE) {							// successful ?
			iRV = psCLI->pasList[iRV].hdlr(psCLI) ;		// yes, execute matching command
			if (psCLI->bEcho && iRV < erSUCCESS) {		// Failed ?
				printfx("%s\n%*.s^\n", psCLI->pcBeg, psCLI->pcParse - psCLI->pcBeg, "") ;
			}
		} else {
			printfx("Command '%.*s' not found!\n", psCLI->pcParse - psCLI->pcBeg, psCLI->pcBeg) ;
		}
#else
		xRulesProcessText(psCLI->pcParse);
#endif
		vCLIreset(psCLI) ;
	} else if (cCmd == CHR_BS) {
		if (psCLI->pcStore > psCLI->pcBeg) {
			--psCLI->pcStore ;
			if (psCLI->bEcho) printfx("%c", CHR_BS);	// printfx("%s", caBS);
		}
	} else if (cCmd == CHR_ESC) {
		if (psCLI->bEcho) printfx("\r%*.s\r", psCLI->u8BSize, "");
		vCLIreset(psCLI) ;
	} else if (isprint(cCmd)) {
		if (psCLI->pcStore < (psCLI->pcBeg + psCLI->u8BSize + 1)) {	// plan for terminating NUL
			*psCLI->pcStore++ = cCmd ;					// store character
		} else {
			printfx("%c", CHR_BEL) ;
		}
	}
	if (psCLI->bEcho && (psCLI->pcStore > psCLI->pcBeg))
		printfx("\r%*.s", psCLI->pcStore - psCLI->pcBeg, psCLI->pcBeg) ;
	return iRV ;
}

void halWL_Report(void) ;
void halWL_ReportLx(void) ;
void vTaskSensorsReport(void) ;
void vControlReportTimeout(void) ;

void vCommandInterpret(int cCmd, bool bEcho) {
	sCLI.bEcho = bEcho ;
	halVARS_ReportFlags(0) ;
	if (cCmd == 0) return ;
	if (sCLI.bMode)  xCommandBuffer(&sCLI, cCmd) ;
	else {
		switch (cCmd) {
		// ########################### Unusual (possibly dangerous) options
		#if	(!defined(NDEBUG) || defined(DEBUG))
		case CHR_DC4: while(1); break ;					// generate watchdog timeout
		case CHR_NAK: *((char *) 0xFFFFFFFF) = 1; break ;
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
		case CHR_4:
		case CHR_5:
		case CHR_6:
		case CHR_7:
			#if	(HW_VARIANT == HW_AC00) || (HW_VARIANT == HW_AC01)
			xActuatorLoad(cCmd - CHR_0 + 8, 1, 0, 6000, 0, 0) ;
			xActuatorLoad(cCmd - CHR_0, 6, 0, 500, 0, 500) ;

			#elif (HW_VARIANT == HW_WROVERKIT) || (HW_VARIANT == HW_DOITDEVKIT)
			if (cCmd - CHR_0 < halSOC_DIG_OUT) xActuatorLoad(cCmd - CHR_0, 5, 500, 500, 500, 500) ;
			else printfx("%c", CHR_BEL) ;

			#elif (HW_VARIANT == HW_EM1P2)
			if (cCmd - CHR_0 < CALIB_NUM) {
				m90e26Report() ;
				m90e26LoadNVSConfig(0, cCmd - CHR_0) ;
				m90e26LoadNVSConfig(1, cCmd - CHR_0) ;
				m90e26Report() ;
			}
			#endif
			break ;

		case CHR_A:
			#if	(halXXX_XXX_OUT > 0)
			vActuatorsIdent() ;
			#endif
			#if (halHAS_M90E26 > 0)
			vActuatorsIdent() ;
			#endif
			break ;

		case CHR_a:
			#if	(halXXX_XXX_OUT > 0)
			vTaskActuatorReport() ;
			#endif
			break ;
		#endif

		#ifdef	ESP_PLATFORM									// ESP32 Specific options
		case CHR_SOH:	halFOTA_SetBootNumber(1, fotaBOOT_REBOOT) ;		break ;	// c-A
		case CHR_STX:	halFOTA_SetBootNumber(2, fotaBOOT_REBOOT) ;		break ;	// c-B
		case CHR_ETX:	halFOTA_SetBootNumber(3, fotaBOOT_REBOOT) ;		break ;	// c-C

		case CHR_DLE:	// c-P
			sNVSvars.HostMQTT = sNVSvars.HostSLOG = sNVSvars.HostFOTA = sNVSvars.HostCONF = (sNVSvars.HostMQTT==hostPROD) ? hostDEV : hostPROD ;
			BlobsFlag |= varFLAG_HOSTS ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;

		case CHR_DC1:	// c-Q (XON)
			sNVSvars.QoSLevel = (sNVSvars.QoSLevel == QOS0) ? QOS1 :
								(sNVSvars.QoSLevel == QOS1) ? QOS2 : QOS0 ;
			BlobsFlag |= varFLAG_QOSLEVEL ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;

		case CHR_DC2:	// c-R
			halFOTA_RevertToPreviousFirmware(fotaBOOT_REBOOT);
			break;

		case CHR_SYN:	// c-V Delete WIFI & VARS blobs, reboot same firmware
			halFOTA_SetBootNumber(halFOTA_GetBootNumber(),  fotaERASE_WIFI|fotaERASE_VARS|fotaBOOT_REBOOT) ;
			break ;

		case CHR_ETB:	// c-W Deleted VARS blob only, reboot SAME firmware
			halFOTA_SetBootNumber(halFOTA_GetBootNumber(), fotaERASE_VARS|fotaBOOT_REBOOT) ;
			break ;

		case CHR_b: {
			#define	blobBUFFER_SIZE			1024
			uint8_t * pBuffer = pvRtosMalloc(blobBUFFER_SIZE) ;
			size_t	SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_PART, pBuffer, &SizeBlob) ;
			SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_WIFI, pBuffer, &SizeBlob) ;
			SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_VARS, pBuffer, &SizeBlob) ;
			#if	(HW_VARIANT == HW_EM1P2)
				SizeBlob = blobBUFFER_SIZE ;
				halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, pBuffer, &SizeBlob) ;
			#endif
			vRtosFree(pBuffer) ;
			break ;
		}
		case CHR_p: halFOTA_ReportPartitions(); break ;
		#endif

		// ############################ Normal (non-dangerous) options

		case CHR_B:
			xRtosSetStatus(flagAPP_RESTART);
			break;
		case CHR_C:
			#if	(halHAS_ONEWIRE > 0)
			++OWflags.Level ;
			IF_PRINT(debugLEVEL, "Level = %u\n", OWflags.Level) ;
			#endif
			break;
		case CHR_D:
			#if	(halHAS_DS18X20 > 0)
			OWP_DS18X20Ai1(NULL) ;
			OWP_ScanAlarmsFamily(OWFAMILY_28) ;
			#endif
			break ;
		case CHR_I:
			#if	(SW_AEP == 1)
			vSW_ReRegister();
			#elif (SW_AEP == 2)
			vTB_ReRegister();
			#endif
			break ;
		case CHR_O:
			#if	(halHAS_ONEWIRE > 0)
			OWP_Report() ;
			#endif
			break ;
		case CHR_T:
			vSysTimerShow(0xFFFFFFFF) ;
			break ;
		case CHR_U:
			xRtosSetStatus(flagAPP_UPGRADE) ;
			break ;

		case CHR_d:
			#if	(halHAS_M90E26 > 0)
			m90e26Report() ;
			#endif
			#if	(halHAS_SSD1306 > 0)
			ssd1306Report() ;
			#endif
			#if	(halHAS_MCP342X > 0)
			mcp342xReportAll() ;
			#endif
			break ;
		case CHR_f:
			halVARS_ReportFlags(1) ;
			break ;
		case CHR_h:
			printfx(HelpMessage) ;
			break ;
		case CHR_i:
			#if	(configUSE_IDENT == 1)
			vID1_ReportAll();
			#elif (configUSE_IDENT == 2)
			vID2_ReportAll();
			#endif
			break ;
		case CHR_l:
			halVARS_ReportGeoloc() ;
			break;
		case CHR_m:
			vRtosReportMemory();
			break;
		case CHR_n:
			xNetReportStats();
			break;
		case CHR_o:
			vOptionsShow();
			break;
		case CHR_r:
			vRulesDecode();
			break;
		case CHR_s:
			vTaskSensorsReport();
			break;
		case CHR_t:
			xRtosReportTasks(makeMASKFLAG(0,0,1,1,1,1,1,1,1,0x007FFFFF), NULL, 0);
			break;
		case CHR_v:
			halMCU_Report() ;
			halVARS_ReportFirmware() ;
			halWL_ReportLx() ;
			vSyslogReport() ;
			IF_EXEC_0(configCONSOLE_HTTP == 1, vHttpReport) ;
			IF_EXEC_0(configCONSOLE_TELNET == 1, vTelnetReport) ;
			#if	(SW_AEP == 1)
			#include	"task_sitewhere.h"
			vSW_Report() ;
			#elif (SW_AEP == 2)
			#include	"task_thingsboard.h"
			vTB_Report() ;
			#endif
			halVARS_ReportSystem() ;
			vControlReportTimeout() ;
			break ;
		case CHR_w:
			halWL_Report();
			break;

		case CHR_Z:
		case CHR_z:
			vCLIreset(&sCLI);
			sCLI.bMode = 1;
			break;
		default:
			printfx("key=0x%03X\r", cCmd);
		}
	}
	halVARS_ReportFlags(0) ;
}

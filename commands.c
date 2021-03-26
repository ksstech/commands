/*
 * commands.c - command interpreter
 */

#include	"hal_variables.h"
#include	"commands.h"

#include	"FreeRTOS_Support.h"						// freertos statistics x_complex_vars x_struct_unions x_time x_definitions stdint time
#include	"actuators.h"

#include	"task_sitewhere.h"
#include	"ident1.h"

#include	"task_thingsboard.h"
#include	"ident2.h"

#include	"x_http_server.h"
#include	"x_string_general.h"						// xstrncmp()
#include	"x_string_to_values.h"
#include	"x_errors_events.h"
#include	"x_builddefs.h"
#include 	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"
#include	"x_telnet_server.h"

#include	"hal_usart.h"
#include 	"hal_mcu.h"									// halMCU_Report()
#include	"hal_fota.h"
#include	"hal_storage.h"

// external modules that offer commands
#include	"hal_network_cmds.h"						// x_struct_union x_time x_definitions stdint.h time.h
#include	"paho_mqtt.h"

#if		(configUSE_RULES > 0)
	#include	"rules_decode.h"
	#include	"rules_parse_text.h"
#endif

#if		(halHAS_SSD1306 > 0)
	#include	"ssd1306.h"
#endif

#if		(halHAS_M90E26 > 0)
	#include	"m90e26_cmds.h"
#endif

#if		(halHAS_DS18X20 > 0)
	#include	"ds18x20_cmds.h"
#endif

#if		(halHAS_PCA9555 > 0)
	#include	"pca9555.h"
#endif

#if		(halHAS_ONEWIRE > 0)
	#include	"onewire_platform.h"
#endif

#include	<string.h>

#define	debugFLAG					0xC000

#define	debugCMND					(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Macros ################################################

#define	bufferMEMDUMP_SIZE			(1 * KILO)
#define	configBUILD_LONG_COMMAND	1

#define	cliBUF_SIZE					128

// ############################### Forward function declarations ##################################

int32_t CmndPEEK(cli_t * psCLI) ;

// #################################### Public variables ##########################################


// ##################################### Local variables ##########################################

cmnd_t sCLIlist[] = {
	{ "PEEK", CmndPEEK }, { "WIFI", CmndWIFI }, { "NWMO", CmndNWMO }, { "MQTT", CmndMQTT },
	{ "CMND", CmndCMND },
#if		(HW_VARIANT == HW_EM1P2)
	{ "M90C", CmndM90C }, { "M90D", CmndM90D }, { "M90L", CmndM90L }, { "M90N", CmndM90N },
	{ "M90O", CmndM90O }, { "M90P", CmndM90P }, { "M90S", CmndM90S }, { "M90Z", CmndM90Z },
#endif

#if		(halHAS_DS18X20 > 0)
	{ "DS18", CmndDS18 },
#endif

#if		(configCONSOLE_UART > 0)
	{ "UART", CmndUART },
#endif

} ;

static	char	CLIbuf[cliBUF_SIZE] = { 0 } ;
static	cli_t	sCLI ;

static const char	HelpMessage[] = {
	"Single character commands\n"
#if		(!defined(NDEBUG) || defined(DEBUG))
	"\tre(B)oot\n"
	"\t(F)lag changes dis/enable\n"
	"\t(T)imer/Scatter Info\n"
	"\t(U)pgrade Firmware\n"

	#if	(configHAL_XXX_XXX_OUT > 0)
	"\t(0-7) Trigger actuator channel 'x'\n"
	"\t(A)ctuators reload\n"
	#endif

	#if	(halHAS_M90E26 > 0) || (halHAS_M90E36 > 0)
	"\t(A)utomatic adjustment\n"
		#if	(halHAS_M90E36 > 0)
		"\t    Calibrate M90E26's\n"
		#endif
		#if	(halHAS_M90E36 > 0)
		"\t    Calibrate M90E36's\n"
		#endif
	"\t(0-2) load predefined config 'x'\n"
	#endif

	#if	(halHAS_DS248X > 0)
	"\t(D)ebug 1W Channels\n"
	"\t(c)DS248X flags level (0-1-2-3-0) increment\n"
	#endif

	#if	defined(ESP_PLATFORM)
	"\tc-T Generate WatchDog timeout\n"
	"\tc-U generate Invalid memory access crash\n"
	#endif
#endif

#if		defined(ESP_PLATFORM)
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
	"\tc-V Reboot current FW as APSTA (delete WIFI & VAR blobs)\n"
#endif

// ##################### non lethal command options

	"\t(b)lob report\n"
	"\t(f)lags Status\n"
	"\t(h)elp screen display\n"
	"\t(l)ocation info\n"
	"\t(m)emory info\n"
	"\t(n)etwork (IP4) info\n"
	"\t(r)ules display\n"
	"\t(s)ensors statistics\n"
	"\t(t)asks statistics\n"
	"\t(v)erbose system info\n"
	"\t(w)ifi Stats\n"

#if		(configHAL_XXX_XXX_OUT > 0)
	"\t(a)ctuators status\n"
#endif


#if		(halHAS_M90E26 > 0)
	"\t ### M90E26 ###\n"
	"\t(d)ebug M90E26 config\n"
	"\t(d)ebug SSD1306 config\n"
#endif

#if		(configUSE_IDENT == 1)
	"\t(I)nit rules & ident\n"
	"\t(i)dent table\n"
#endif

#if		(halHAS_ONEWIRE > 0)
	"\t ### 1-WIRE ###\n"
	#if	(halHAS_DS248X > 0)
	"\t(d)ebug DS248X Configuration\n"
	#endif
		"\t(o)newire info\n"
#endif

#if		defined(ESP_PLATFORM)
	"\t(p)artitions report\n"
#endif

	"\nExtended commands, prefix with 'z'\n"
	"\tMQTT addr port\t\t{en/disable local broker}\n"
	"\tWIFI ssid pswd\t\t{set wifi credentials}\n"
	"\tNWMO mode (0->3)\t\{set network mode}\n"
	"\tRULE {sense|mode|rule text to decode}\n"

#if		(configCONSOLE_UART > 0)
	"\tUART chan speed\t\t{set baudrate}\n"
#endif

#if		(!defined(NDEBUG) || defined(DEBUG))
	"\tPEEK addr length\t\{dump section of memory}\n"

	#if	(halHAS_DS18X20 > 0)
	"\tDS18 {RDT|RDSP|WRSP|MODE} {Lchan} {Lo Hi Res}\n"
	#endif

	#if	(halHAS_M90E26 > 0)
	"\tM90C chan reg value\t{configure a register [+CRC]}\n"
	"\tM90D\t\t\t{delete the NVS blob}\n"
	"\tM90L chan value\t\t{set Live gain}\n"
	"\tM90N chan value\t\t{set Neutral gain}\n"
	"\tM90O chan\t\t{Current OFFSET adjust}\n"
	"\tM90P chan\t\t{Power OFFSET adjust}\n"
	"\tM90S chan index\t\t{save config to blob #}\n"
	"\tM90Z chan\t\t{reset to defaults}\n"
	#endif
#endif
	"\n"
} ;

// ############################### General status reporting functions ##############################


// ############################### UART/TNET/HTTP Command interpreter ##############################

int32_t CmndParseAddrMEM(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (px_t) pAddr, vfUXX, vs32B, sepSPACE) ;
	IF_PRINT(debugCMND && pTmp == pcFAILURE, " erFAILURE") ;
	IF_PRINT(debugCMND && !halCONFIG_inFLASH(*pAddr), " erRANGE") ;
	if (pTmp != pcFAILURE && (halCONFIG_inMEM(*pAddr) == 0)) {
		psCLI->pcParse = pTmp ;
		return erSUCCESS ;
	}
	return erFAILURE ;
}

int32_t CmndParseAddrFLASH(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (px_t) pAddr, vfUXX, vs32B, sepSPACE) ;
	IF_PRINT(debugCMND && pTmp == pcFAILURE, " erFAILURE") ;
	IF_PRINT(debugCMND && !halCONFIG_inFLASH(*pAddr), " erRANGE") ;
	if (pTmp != pcFAILURE && halCONFIG_inFLASH(*pAddr)) {
		psCLI->pcParse = pTmp ;
		return erSUCCESS ;
	}
	return erFAILURE ;
}

int32_t CmndParseAddrSRAM(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (px_t) pAddr, vfUXX, vs32B, sepSPACE) ;
	IF_PRINT(debugCMND && pTmp == pcFAILURE, " erFAILURE") ;
	IF_PRINT(debugCMND && !halCONFIG_inSRAM(*pAddr), " erRANGE") ;
	if ((pTmp != pcFAILURE) && halCONFIG_inSRAM(*pAddr)) {
		psCLI->pcParse = pTmp ;
		return erSUCCESS ;
	}
	return erFAILURE ;
}

int32_t CmndPEEK(cli_t * psCLI) {
	void *		Addr ;
	uint32_t	Size ;
	int32_t iRV = CmndParseAddrMEM(psCLI, &Addr) ;
	if (iRV != erFAILURE) {
		char * pTmp = pcStringParseValueRange(psCLI->pcParse, (px_t) &Size, vfUXX, vs32B, sepSPACE, (x32_t) 1, (x32_t) 1024) ;
		if (pTmp != pcFAILURE) {
			printfx("PEEK %p %u\n%'+B", Addr, Size, Size, Addr) ;
			psCLI->pcParse = pTmp ;
			return erSUCCESS ;
		}
	}
	return iRV ;
}

int32_t	xCLImatch(cli_t * psCLI) {
	for (int32_t Idx = 0; Idx < psCLI->u8LSize; ++Idx) {
		size_t Len = strnlen(psCLI->pasList[Idx].cmnd, SIZEOF_MEMBER(cmnd_t, cmnd)) ;
		if (xstrncmp(psCLI->pcParse, psCLI->pasList[Idx].cmnd, Len, 0) == 1) {
			psCLI->pcParse += Len ;
			return Idx ;
		}
	}
	return erFAILURE ;
}

void	vCLIreset(cli_t * psCLI) {
	psCLI->pcStore	= psCLI->pcParse	= psCLI->pcBeg = CLIbuf ;
	sCLI.u8BSize	= cliBUF_SIZE ;
	sCLI.pasList	= sCLIlist ;
	sCLI.u8LSize	= NUM_OF_MEMBERS(sCLIlist) ;
	psCLI->bMode	= 0 ;
	memset(psCLI->pcBeg, 0, psCLI->u8BSize) ;
}

void	vCLIinit(void) { vCLIreset(&sCLI) ; }

static	char caBS[] = { CHR_BS, CHR_SPACE, CHR_BS, CHR_NUL } ;

int32_t	xCommandBuffer(cli_t * psCLI, int32_t cCmd) {
	int32_t iRV = erSUCCESS ;
	if (cCmd == CHR_CR || cCmd == CHR_NUL) {			// terminating char received
		*psCLI->pcStore	= CHR_NUL ;
		if (psCLI->bEcho) {
			printfx("\n") ;
		}
		iRV = xCLImatch(psCLI) ;						// try to find matching command
		if (iRV > erFAILURE) {							// successful ?
			iRV = psCLI->pasList[iRV].hdlr(psCLI) ;		// yes, execute matching command
			if (psCLI->bEcho && iRV < erSUCCESS) {		// Failed ?
				printfx("%s\n%*.s^\n", psCLI->pcBeg, psCLI->pcParse - psCLI->pcBeg, "") ;
			} else {
				// command was successful
			}
		} else {
			printfx("Command '%.*s' not found!\n", psCLI->pcParse - psCLI->pcBeg, psCLI->pcBeg) ;
		}
		vCLIreset(psCLI) ;
	} else if (cCmd == CHR_BS && psCLI->pcStore > psCLI->pcBeg) {
		--psCLI->pcStore ;
		if (psCLI->bEcho) {
			printfx("%s", caBS) ;
		}
	} else if (cCmd == CHR_ESC) {
		if (psCLI->bEcho) {
			printfx("\r%*.s\r", psCLI->u8BSize, "") ;
		}
		vCLIreset(psCLI) ;
	} else if (isprint(cCmd)) {
		if (psCLI->pcStore < (psCLI->pcBeg + psCLI->u8BSize + 1)) {	// plan for terminating NUL
			*psCLI->pcStore++ = cCmd ;					// store character
		} else {
			printfx("%c", CHR_BEL) ;
		}
	}
	if (psCLI->bEcho && (psCLI->pcStore > psCLI->pcBeg)) {
		printfx("\r%*.s", psCLI->pcStore - psCLI->pcBeg, psCLI->pcBeg) ;
	}
	return iRV ;
}

void	halWL_Report(void) ;
void	halWL_ReportLx(void) ;
void	vTaskSensorsReport(void) ;
void	vControlReportTimeout(void) ;

void	vCommandInterpret(int32_t cCmd, bool bEcho) {
	sCLI.bEcho = bEcho ;
	halVARS_ReportFlags(&sCLI) ;
	if (cCmd == CHR_NUL) {
		return ;
	}
	if (sCLI.bMode) {
		xCommandBuffer(&sCLI, cCmd) ;
	} else {
		switch (cCmd) {
	// ########################### Unusual (possibly dangerous) options

#if		(!defined(NDEBUG) || defined(DEBUG))			// Diagnostic related options
		case CHR_DC4:
			while(1) ; 									// generate watchdog timeout
			break ;
		case CHR_NAK:
			*((char *) 0xFFFFFFFF) = 1 ; 				// invalid memory access
			break ;
		case CHR_0:
		case CHR_1:
		case CHR_2:
		case CHR_3:
		case CHR_4:
		case CHR_5:
		case CHR_6:
		case CHR_7:
	#if		(HW_VARIANT == HW_AC00) || (HW_VARIANT == HW_AC01)
			xActuatorLoad(cCmd - CHR_0 + 8, 1, 0, 6000, 0, 0) ;
			xActuatorLoad(cCmd - CHR_0, 6, 0, 500, 0, 500) ;

	#elif	(HW_VARIANT == HW_WROVERKIT) || (HW_VARIANT == HW_DOITDEVKIT)
			if (cCmd - CHR_0 < configHAL_GPIO_DIG_OUT) {
				xActuatorLoad(cCmd - CHR_0, 5, 500, 500, 500, 500) ;
			} else {
				printfx("%c", CHR_BEL) ;
			}
	#elif	(HW_VARIANT == HW_EM1P2)
			if (cCmd - CHR_0 < CALIB_NUM) {
				m90e26Report() ;
				m90e26LoadNVSConfig(0, cCmd - CHR_0) ;
				m90e26LoadNVSConfig(1, cCmd - CHR_0) ;
				m90e26Report() ;
			}
	#endif
			break ;

	#if	(HW_VARIANT==HW_AC00 || HW_VARIANT==HW_AC01 || HW_VARIANT==HW_WROVERKIT || HW_VARIANT==HW_DOITDEVKIT)
		case CHR_A:	vActuatorsIdent() ;								break ;
		case CHR_a:	vTaskActuatorReport() ;							break ;
	#endif
#endif

#if		defined(ESP_PLATFORM)							// ESP32 Specific options
		case CHR_SOH:	halFOTA_SetBootNumber(1, fotaBOOT_REBOOT) ;		break ;	// c-A
		case CHR_STX:	halFOTA_SetBootNumber(2, fotaBOOT_REBOOT) ;		break ;	// c-B
		case CHR_ETX:	halFOTA_SetBootNumber(3, fotaBOOT_REBOOT) ;		break ;	// c-C
		case CHR_DLE:															// c-P
			sNVSvars.HostMQTT = sNVSvars.HostSLOG = sNVSvars.HostFOTA = sNVSvars.HostCONF = (sNVSvars.HostMQTT==hostPROD) ? hostDEV : hostPROD ;
			VarsFlag |= varFLAG_HOSTS ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;
		case CHR_DC1:															// c-Q (XON)
			sNVSvars.QoSLevel = (sNVSvars.QoSLevel == QOS0) ? QOS1 :
								(sNVSvars.QoSLevel == QOS1) ? QOS2 : QOS0 ;
			VarsFlag |= varFLAG_QOSLEVEL ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;
		case CHR_DC2:															// c-R
			halFOTA_RevertToPreviousFirmware(fotaBOOT_REBOOT) ;
			break ;
		case CHR_SYN:															// c-V
			halFOTA_SetBootNumber(halFOTA_GetBootNumber(), fotaERASE_WIFI | fotaBOOT_REBOOT | fotaERASE_VARS) ;
			IF_TRACK(debugTRACK, "Reset config & restart") ;
			break ;
		case CHR_b: {
			#define	blobBUFFER_SIZE			1024
			uint8_t * pBuffer = malloc(blobBUFFER_SIZE) ;
			size_t	SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_PART, pBuffer, &SizeBlob) ;
			SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_WIFI, pBuffer, &SizeBlob) ;
			SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_VARS, pBuffer, &SizeBlob) ;
	#if		(HW_VARIANT == HW_EM1P2)
			SizeBlob = blobBUFFER_SIZE ;
			halSTORAGE_ReportBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, pBuffer, &SizeBlob) ;
	#endif
			free(pBuffer) ;
			break ;
		}
		case CHR_p:
			halFOTA_ReportPartitions() ;
			break ;
#endif

		// ############################ Normal (non-dangerous) options

		case CHR_B:
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;
		case CHR_F:
			sNVSvars.fFlags	= sNVSvars.fFlags ? 0 : 1 ;
			VarsFlag |= varFLAG_FLAGS ;
			break ;
		case CHR_T:
			vSysTimerShow(0xFFFFFFFF) ;
			break ;
		case CHR_U:
			xRtosSetStatus(flagAPP_UPGRADE) ;
			break ;
		case CHR_f:
			sCLI.bForce	= 1 ;
			halVARS_ReportFlags(&sCLI) ;
			sCLI.bForce	= 0 ;
			break ;
		case CHR_h:
			printfx(HelpMessage) ;
			break ;
		case CHR_l:
			halVARS_ReportGeoloc() ;
			break ;
		case CHR_m:
			vRtosReportMemory() ;
			break ;
		case CHR_n:
			xNetReportStats() ;
			break ;
		case CHR_r:
			vRulesDecode() ;
			break ;
		case CHR_s:
			vTaskSensorsReport() ;
			break ;
		case CHR_t:
			xRtosReportTasksNew(makeMASKFLAG(0,0,0,0,0,1,1,1,1,1,1,1,0xFFFFF), NULL, 0) ;
			break;
		case CHR_v:
			halMCU_Report() ;
			halVARS_ReportFirmware() ;
			halWL_ReportLx() ;
			vSyslogReport() ;
			IF_EXEC_0(configCONSOLE_HTTP == 1, vHttpReport) ;
			IF_EXEC_0(configCONSOLE_TELNET == 1, vTelnetReport) ;
			IF_EXEC_0(SW_AEP == 1, vSW_Report) ;
			IF_EXEC_0(SW_AEP == 2, vTB_Report) ;
			halVARS_ReportSystem() ;
			vControlReportTimeout() ;
			break ;
		case CHR_w:
			halWL_Report() ;
			break ;
#if		(configBUILD_LONG_COMMAND == 1)
		case CHR_Z:
		case CHR_z:
			vCLIreset(&sCLI) ;
			sCLI.bMode		= 1 ;
			break ;
#endif

#if		(halHAS_ONEWIRE > 0)
	#if	(halHAS_DS18X20 > 0)
		case CHR_D:
			ds18x20ReadConvertAll(NULL) ;
			ds18x20ScanAlarmsAll() ;
			break ;
		case CHR_c:
			++OWflags.Level ;
			break ;
		case CHR_d:
			ds248xReportAll() ;
			break ;
	#endif
		case CHR_o:
			OWPlatformReportAll() ;
			break ;
#endif

#if		(SW_AEP == 1)
		case CHR_I:
			vSW_ReRegister();
			break ;
		case CHR_i:
			vID1_ReportAll() ;
			break ;
#elif	(SW_AEP == 2)
		case CHR_I:
			vTB_ReRegister();
			break ;
		case CHR_i:
			vIdentityReportAll() ;
			break ;
#endif

#if		(halHAS_M90E26 > 0)
		case CHR_d:
			m90e26Report() ;
			ssd1306Report() ;
			break ;
#endif

		default:	printfx("key=0x%03X\r", cCmd) ;
		}
	}
	halVARS_ReportFlags(&sCLI) ;
}

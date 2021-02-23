/*
 * commands.c - command interpreter
 */
#include	"hal_config.h"

#include	"commands.h"

#include	"FreeRTOS_Support.h"						// freertos statistics x_complex_vars x_struct_unions x_time x_definitions stdint time
#include	"paho_support.h"
#include	"actuators.h"
#include	"identity.h"

#include	"x_http_server.h"
#include	"x_string_general.h"						// pcBitMapDecode()
#include	"x_string_to_values.h"
#include	"x_errors_events.h"
#include	"x_builddefs.h"
#include 	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"
#include	"x_telnet_server.h"

#include 	"hal_mcu.h"									// halMCU_Report()
#include	"hal_fota.h"
#include	"hal_storage.h"

// external modules that offer commands
#include	"hal_network_cmds.h"						// x_struct_union x_time x_definitions stdint.h time.h
#if		(SW_AEP == 1)
	#include	"task_sitewhere_cmds.h"					// x_struct_union x_time x_definitions stdint.h time.h
#elif	(SW_AEP == 2)
	#include	"task_thingsboard_cmds.h"				// x_struct_union x_time x_definitions stdint.h time.h
#endif

#include	"hal_usart.h"
#include	"hal_nvs.h"

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

#include	"lwip/init.h"							// LWIP_VERSION_STRING

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
#if		(configUSE_RULES > 0)
	{ "CMND", CmndCMND },
#endif
} ;

static	char	CLIbuf[cliBUF_SIZE] = { 0 } ;
static	cli_t	sCLI ;

static const char	HelpMessage[] = { "Single character commands\n"
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
	#if	!defined(NDEBUG) || defined(DEBUG)
		"\tc-T Generate WatchDog timeout\n"
	"\tc-U generate Invalid memory access crash\n"
	#endif
	"\tc-V Reboot current FW as APSTA (delete WIFI & VAR blobs)\n"
#endif

#if		(configHAL_XXX_XXX_OUT > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t(0-7) Trigger actuator channel 'x'\n"
	"\t(A)ctuators reload\n"

#elif	(halHAS_M90E26 > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t(0-2) load predefined config 'x'\n"
	"\t(A)utomatic adjustment\n"
#endif

	"\tre(B)oot\n"

	"\t(D)ebug various\n"
#if		(halHAS_M90E26 > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t    Calibrate M90E26's\n"
#endif
#if		(halHAS_M90E36 > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t    Calibrate M90E36's\n"
#endif
#if		(halHAS_DS248X > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t    Scan 1W Channels\n"
#endif

	"\t(F)lag changes dis/enable\n"
	"\t(I)nit rules & ident\n"
	"\t(T)imer/Scatter Info\n"
	"\t(U)pgrade Firmware\n"

// ##################### non lethal command options

#if		(configHAL_XXX_XXX_OUT > 0)
	"\t(a)ctuators status\n"
#endif
	"\t(b)lob report\n"

#if	(halHAS_DS248X > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\t(c)DS248X flags level (0-1-2-3-0) increment\n"
#endif

	"\t(d)ebug reports\n"
#if		(halHAS_DS248X > 0)
	"\t    DS248X Configuration\n"

#elif	(halHAS_M90E26 > 0)
	"\t    M90E26 config\n"
	"\t    SSD1306 config\n"
#endif

	"\t(f)lags Status\n"
	"\t(h)elp screen display\n"
#if		(configUSE_IDENT == 1)
	"\t(i)dent table\n"
#endif
	"\t(l)ocation info\n"
	"\t(m)emory info\n"
	"\t(n)etwork (IP4) info\n"
#if		(halHAS_ONEWIRE > 0)
	"\t(o)newire info\n"
#endif
#if		defined(ESP_PLATFORM)
	"\t(p)artitions report\n"
#endif
	"\t(r)ules display\n"
	"\t(s)ensors statistics\n"
	"\t(t)asks statistics\n"
	"\t(v)erbose system info\n"
	"\t(w)ifi Stats\n"
	"Extended commands, prefix with 'z'\n"
#if		(halHAS_DS18X20 > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\tDS18 {RDT|RDSP|WRSP|MODE} {Lchan} {Lo Hi Res}\n"
#endif
#if		(halHAS_M90E26 > 0) && (!defined(NDEBUG) || defined(DEBUG))
	"\tM90C chan reg value\t{configure a register [+CRC]}\n"
	"\tM90D\t\t\t{delete the NVS blob}\n"
	"\tM90L chan value\t\t{set Live gain}\n"
	"\tM90N chan value\t\t{set Neutral gain}\n"
	"\tM90O chan\t\t{Current OFFSET adjust}\n"
	"\tM90P chan\t\t{Power OFFSET adjust}\n"
	"\tM90S chan index\t\t{save config to blob #}\n"
	"\tM90Z chan\t\t{reset to defaults}\n"
#endif
	"\tMQTT addr port\t\t{en/disable local broker}\n"
#if		 (!defined(NDEBUG) || defined(DEBUG))
	"\tPEEK addr length\t\{dump section of memory}\n"
#endif
	"\tWIFI ssid pswd\t\t{set wifi credentials}\n"
	"\tNWMO mode (0->3)\t\{set network mode}\n"
#if		(configCONSOLE_UART > 0)
	"\tUART chan speed\t\t{set baudrate}\n"
#endif
#if		(configUSE_RULES > 0)
	"\tRULE {sense|mode|rule text to decode}\n"
#endif
		"\n"
} ;

// Completely static values, not persisted in NVS
const char * const FlagNames[24] = {
	[0]		= "L1",
	[1]		= "L2sta",
	[2]		= "L3sta",
	[3]		= "L2ap",
	[4]		= "L3ap",
	[5]		= "SNTP",
	[6]		= "MQTT",
	[7]		= "TNETS",
	[8]		= "TNETC",
	[9]		= "HTTPS",
	[10]	= "HTTPC",
	[11]	= "SLOG",
	[12]	= "",
	[13]	= "",
	[14]	= "",
	[15]	= "",
	[16]	= "RVRT",
	[17]	= "TIMER",
	[18]	= "UpGRD",
	[19]	= "RSTRT",
	[20]	= "RGSTR",
	[21]	= "RULES",
	[22]	= "IDENT",
	[23]	= "i2cOK",
} ;

// ############################### General status reporting functions ##############################

void	vControlReport(void) {
	printfx("%CFW%C\t" VER_INFO " UART#%d %u Rx=%u Tx=%u\n", xpfSGR(attrRESET, colourFG_CYAN, 0, 0), xpfSGR(attrRESET, 0, 0, 0),
		configSTDIO_UART_CHAN, usartInfo[configSTDIO_UART_CHAN].Speed,
		usartInfo[configSTDIO_UART_CHAN].RxSize, usartInfo[configSTDIO_UART_CHAN].TxSize) ;
}

void	vIrmacosReport(void) {
	printfx_lock() ;
	printfx_nolock("%CMisc%C\tSe=%u [St=%u]  M=%u  O=%u",
		xpfSGR(attrRESET, colourFG_CYAN, 0, 0), xpfSGR(attrRESET, 0, 0, 0),
		SenseCount, StatsCount, ModeCount, OtherCount) ;
#if		(configUSE_RULES == 1) && (configUSE_IDENT == 0)
	printfx_nolock("  R=%u/%u[%u]\n", RulesCount, RulesSizeReq, RulesTableSize) ;
#elif	(configUSE_RULES == 1) && (configUSE_IDENT == 1)
	printfx_nolock("  R=%u/%u[%u]", RulesCount, RulesSizeReq, RulesTableSize) ;
	printfx_nolock("  I=%u/%u[%u]\n", IdentCount, IdentSizeReq, IdentTableSize) ;
#endif

#if		defined(ESP_PLATFORM)
	printfx_nolock("\tcntVars=%u  cntWifi=%u  Reboots=%u\n",
		sNVSvars.countVars, sNVSvars.countWifi, halCONFIG_RebootCounterRead()) ;
#endif
#if		(halHAS_PCA9555 > 0)
	printfx_nolock("\tPCA9555 Checks  OK=%d  Fail=%d\n", pcaSuccessCount, pcaResetCount) ;
#endif
	printfx_unlock() ;
}

void	vFlagsReport(cli_t * psCLI) {
	static	uint32_t Fprv = 0 ;
	if (sNVSvars.fFlags || psCLI->bForce) {
		uint32_t	Fcur = xEventGroupGetBits(xEventStatus) ;
		if (Fprv != Fcur || psCLI->bForce) {
			// Mask is bottom 24 bits max FreeRTOS limit
			char * pcBuf = pcBitMapDecodeChanges(Fprv, Fcur, 0x00FFFFFF, FlagNames) ;
			SL_LOG(SL_SEV_WARNING, pcBuf) ;
			free(pcBuf) ;
			Fprv = Fcur ;
		}
	}
}

void	vGeoLocReport(void) {
	printfx_lock() ;
	printfx_nolock("%CLocInfo%C\t", xpfSGR(attrRESET, colourFG_CYAN, 0, 0), xpfSGR(attrRESET, 0, 0, 0)) ;
	printfx_nolock("Lat=%f  Lon=%f  Alt=%f  Acc=%f  Res=%f\n", sNVSvars.GeoLocation[Latitude], sNVSvars.GeoLocation[Longitude],
			sNVSvars.GeoLocation[Altitude], sNVSvars.GeoLocation[Accuracy], sNVSvars.GeoLocation[Resolution]) ;
	printfx_nolock("%CTZ Info%C\t", xpfSGR(attrRESET, colourFG_CYAN, 0, 0), xpfSGR(attrRESET, 0, 0, 0)) ;
	printfx_nolock("TZid=%s  TZname=%s  Ofst=%ds  DST=%ds\n", sNVSvars.TimeZoneId, sNVSvars.TimeZoneName, sNVSvars.timezone, sNVSvars.daylight) ;
	printfx_unlock() ;
}

// ############################### UART/TNET/HTTP Command interpreter ##############################

int32_t CmndParseAddrMEM(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (p32_t) pAddr, vfUXX, vs32B, sepSPACE) ;
	IF_PRINT(debugCMND && pTmp == pcFAILURE, " erFAILURE") ;
	IF_PRINT(debugCMND && !halCONFIG_inFLASH(*pAddr), " erRANGE") ;
	if (pTmp != pcFAILURE && (halCONFIG_inMEM(*pAddr) == false)) {
		psCLI->pcParse = pTmp ;
		return erSUCCESS ;
	}
	return erFAILURE ;
}

int32_t CmndParseAddrFLASH(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (p32_t) pAddr, vfUXX, vs32B, sepSPACE) ;
	IF_PRINT(debugCMND && pTmp == pcFAILURE, " erFAILURE") ;
	IF_PRINT(debugCMND && !halCONFIG_inFLASH(*pAddr), " erRANGE") ;
	if (pTmp != pcFAILURE && halCONFIG_inFLASH(*pAddr)) {
		psCLI->pcParse = pTmp ;
		return erSUCCESS ;
	}
	return erFAILURE ;
}

int32_t CmndParseAddrSRAM(cli_t * psCLI, void ** pAddr) {
	char * pTmp = pcStringParseValue(psCLI->pcParse, (p32_t) pAddr, vfUXX, vs32B, sepSPACE) ;
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
		char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &Size, vfUXX, vs32B, sepSPACE, (x32_t) 1, (x32_t) 1024) ;
		if (pTmp != pcFAILURE) {
			PRINT("PEEK %p %u\n%'+b", Addr, Size, Size, Addr) ;
			psCLI->pcParse = pTmp ;
			return erSUCCESS ;
		}
	}
	return iRV ;
}

int32_t	xCLImatch(cli_t * psCLI) {
	for (int32_t Idx = 0; Idx < psCLI->u8LSize; ++Idx) {
		size_t Len = strnlen(psCLI->pasList[Idx].cmnd, SIZEOF_MEMBER(cmnd_t, cmnd)) ;
		if (xstrncmp(psCLI->pcParse, psCLI->pasList[Idx].cmnd, Len, false) == true) {
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
		if (psCLI->bEcho)
			printfx("\n") ;
		iRV = xCLImatch(psCLI) ;						// try to find matching command
		if (iRV != erFAILURE) {							// successful ?
			iRV = psCLI->pasList[iRV].hdlr(psCLI) ;		// yes, execute matching command
			if (psCLI->bEcho && iRV != erSUCCESS)		// successful ?
				printfx("%s\n%*.s^\n", psCLI->pcBeg, psCLI->pcParse - psCLI->pcBeg, "") ;
		} else
			printfx("Command '%.*s' not found!\n", psCLI->pcParse - psCLI->pcBeg, psCLI->pcBeg) ;
		vCLIreset(psCLI) ;

	} else if (cCmd == CHR_BS && psCLI->pcStore > psCLI->pcBeg) {
		--psCLI->pcStore ;
		if (psCLI->bEcho)
			printfx("%s", caBS) ;

	} else if (cCmd == CHR_ESC) {
		if (psCLI->bEcho)
			printfx("\r%*.s\r", psCLI->u8BSize, "") ;
		vCLIreset(psCLI) ;

	} else if (isprint(cCmd)) {
		if (psCLI->pcStore < (psCLI->pcBeg + psCLI->u8BSize + 1))	// plan for terminating NUL
			*psCLI->pcStore++ = cCmd ;					// store character
		else
			printfx("%c", CHR_BEL) ;

	}
	if (psCLI->bEcho && (psCLI->pcStore > psCLI->pcBeg))
		printfx("\r%*.s", psCLI->pcStore - psCLI->pcBeg, psCLI->pcBeg) ;
	return iRV ;
}

void	halWL_Report(void) ;
void	halWL_ReportLx(void) ;
void	vTaskSensorsReport(void) ;
void	vControlReportTimeout(void) ;

void	vCommandInterpret(int32_t cCmd, bool bEcho) {
	sCLI.bEcho = bEcho ;
	vFlagsReport(&sCLI) ;
	if (cCmd == CHR_NUL)
		return ;
	if (sCLI.bMode) {
		xCommandBuffer(&sCLI, cCmd) ;
	} else {
		switch (cCmd) {
	// ########################### Unusual (possibly dangerous ) options

#if		defined(ESP_PLATFORM)
		case CHR_SOH:	halFOTA_SetBootNumber(1, fotaBOOT_REBOOT) ;		break ;	// c-A
		case CHR_STX:	halFOTA_SetBootNumber(2, fotaBOOT_REBOOT) ;		break ;	// c-B
		case CHR_ETX:	halFOTA_SetBootNumber(3, fotaBOOT_REBOOT) ;		break ;	// c-C
		case CHR_DLE:															// c-P
			sNVSvars.HostMQTT = sNVSvars.HostSLOG = sNVSvars.HostFOTA = sNVSvars.HostCONF = (sNVSvars.HostMQTT==hostPROD) ? hostDEV : hostPROD ;
			VarsFlag |= varFLAG_HOSTS ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;
		case CHR_DC1:															// c-Q (XON)
			sNVSvars.QoSLevel = (sNVSvars.QoSLevel == QOS0) ? QOS1 : (sNVSvars.QoSLevel == QOS1) ? QOS2 : QOS0 ;
			VarsFlag |= varFLAG_QOSLEVEL ;
			xRtosSetStatus(flagAPP_RESTART) ;
			break ;
		case CHR_DC2:															// c-R
			halFOTA_RevertToPreviousFirmware(fotaBOOT_REBOOT) ;
			break ;
		case CHR_SYN:															// c-V
			halFOTA_SetBootNumber(halFOTA_GetBootNumber(), fotaERASE_WIFI | fotaBOOT_REBOOT | fotaERASE_VARS) ;
			IF_SL_INFO(debugTRACK, "Reset config & restart") ;
			break ;
#endif

#if		(!defined(NDEBUG) || defined(DEBUG))
		case CHR_DC4:	while(1) ; break ;										// generate watchdog timeout
		case CHR_NAK:	*((char *) 0xFFFFFFFF) = 1 ; break ;					// generate invalid memory access restart
#endif

	// ################################## Diagnostic related options

#if		(!defined(NDEBUG) || defined(DEBUG))
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
				PRINT("%c", CHR_BEL) ;
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
	#endif
#endif

		case CHR_B:	xRtosSetStatus(flagAPP_RESTART) ;				break ;
#if		(halHAS_DS18X20 > 0)
//		case CHR_D:	ds18x20ReadConvertAll(NULL) ;					break ;
		case CHR_D:	ds18x20ScanAlarmsAll() ;						break ;
#endif
		case CHR_F:
			sNVSvars.fFlags	= sNVSvars.fFlags ? 0 : 1 ;
			VarsFlag |= varFLAG_FLAGS ;
			break ;
#if		(SW_AEP == 1)
		case CHR_I:	{ void vSiteWhereReRegister(void); vSiteWhereReRegister();	break; }
#elif	(SW_AEP == 2)
		case CHR_I:	{ void vThingsBoardReRegister(void); vThingsBoardReRegister(); break; }
#endif
		case CHR_T:	vSysTimerShow(0xFFFFFFFF) ; 					break ;
		case CHR_U:	xRtosSetStatus(flagAPP_UPGRADE) ;				break ;

	// ############################ Normal (non-dangerous) options

	#if	(configHAL_XXX_XXX_OUT > 0)
		case CHR_a:	vTaskActuatorReport() ;							break ;
	#endif

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

	#if	(halHAS_DS248X > 0)
		case CHR_c:	++OWflags.Level ; break ;
	#endif

		case CHR_d:
#if		(halHAS_DS248X > 0)
			ds248xReportAll() ;
#endif
#if		(halHAS_M90E26 > 0)
			m90e26Report() ;
#endif
#if		(halHAS_SSD1306 > 0)
			ssd1306Report() ;
#endif
			break ;

		case CHR_f:
			sCLI.bForce	= 1 ;
			vFlagsReport(&sCLI) ;
			sCLI.bForce	= 0 ;
			break ;

		case CHR_h:	PRINT(HelpMessage) ;			break ;

	#if		(configUSE_IDENT == 1)
		case CHR_i:	vIdentityReportAll() ;							break ;
	#endif

		case CHR_l:	vGeoLocReport() ;				break ;
		case CHR_m:	vRtosReportMemory() ;			break ;
		case CHR_n:	xNetReportStats() ;				break ;

	#if	(halHAS_ONEWIRE > 0)
		case CHR_o:	OWPlatformReportAll() ;			break ;
	#endif

	#if		defined(ESP_PLATFORM)
		case CHR_p:	halFOTA_ReportPartitions() ;	break ;
	#endif

		case CHR_r:	vRulesDecode() ;				break ;
		case CHR_s:	vTaskSensorsReport() ;			break ;
		case CHR_t:	xRtosReportTasksNew(makeMASKFLAG(0,0,0,0,0,1,1,1,1,1,1,1,0xFFFFF), NULL, 0) ; 	break;

		case CHR_v:
			halMCU_Report() ;
			vControlReport() ;
			halWL_ReportLx() ;
			vSyslogReport() ;
			IF_EXEC_0(configCONSOLE_HTTP == 1, vHttpReport) ;
			IF_EXEC_0(configCONSOLE_TELNET == 1, vTelnetReport) ;
	#if		(SW_AEP == 1)
			void vSiteWhereReport(void) ; vSiteWhereReport() ;
	#elif	(SW_AEP == 2)
			void vThingsBoardReport(void) ; vThingsBoardReport() ;
	#endif
			vIrmacosReport() ;
			vControlReportTimeout() ;
			break ;

		case CHR_w:	halWL_Report() ;				break ;

	#if		(configBUILD_LONG_COMMAND == 1)
		case CHR_Z:
		case CHR_z:
			vCLIreset(&sCLI) ;
			sCLI.bMode		= 1 ;
			break ;
	#endif

		default:	PRINT("key=0x%03X\r", cCmd) ;
		}
	}
	vFlagsReport(&sCLI) ;
}

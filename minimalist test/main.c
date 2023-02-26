//demo for RTC vs. POSC issues on the following chips
//
//dspic33fj128GP804, dspic33fj128GP802, dspic33fj128GP204, dspic33fj128GP202
//dspic33fj64GP804,  dspic33fj64GP802,  dspic33fj64GP204,  dspic33fj64GP202

//test configuration
#define USE_POSC					//use POSC. If commented out, use FRC
//end test configuration

#if defined(__XC16__)
#include <xc.h>						//header file for the device
//************XC16********************
//config words settings
#if defined(USE_POSC)
#pragma config FNOSC = PRI			//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV16, LPRCDIVN
#else
#pragma config FNOSC = FRC			//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV16, LPRCDIVN
#endif
#pragma config POSCMD = XT			//EC, XT, HS, NONE
#pragma config BWRP = WRPROTECT_OFF
#pragma config BSS =  NO_BOOT_CODE
#pragma config RBS =  NO_BOOT_RAM
#pragma config SWRP = WRPROTECT_OFF
#pragma config SSS =  NO_SEC_CODE
#pragma config RSS =  NO_SEC_RAM
#pragma config GWRP = OFF
#pragma config IESO = OFF
#pragma config OSCIOFNC=OFF			//ON: OSC2 as digital IO, OFF: OSC2 has digital output
#pragma config IOL1WAY =OFF			//off: allow multiple reconfiguration, on: allow only one re-configuration
#pragma config FCKSM =  CSECMD		//CSECME, CSECMD, CSDCMD
#pragma config WDTPOST =PS32768
#pragma config WDTPRE = PR128		//PR32, PR128
#pragma config WINDIS = OFF			//off: watchdog timer in non-window mode
#pragma config FWDTEN = OFF
#pragma config FPWRT =  PWR128
#pragma config ALTI2C = OFF
#pragma config ICS =    PGD1		//PGD1, PGD2, PGD3
#pragma config JTAGEN = OFF
//end of xc16

#else								//C30
#include <p33fxxxx.h>				//header file for the device
//*************C30**********************/
//config words settings

_FOSCSEL(
//**   Oscillator Source Selection:
#if defined(USE_POSC)
		FNOSC_PRI &          //Fast RC oscillator
#else
		FNOSC_FRC &			//Primary oscillator (XT, HS, EC)
#endif
//**     FNOSC_FRCPLL         Fast RC oscillator w/ divide and PLL
//     FNOSC_PRIPLL &         //Primary oscillator (XT, HS, EC) w/ PLL
//     FNOSC_SOSC &          //Secondary oscillator
//     FNOSC_LPRC &          //Low power RC oscillator
//     FNOSC_FRCDIV16 &      //Fast RC oscillator w/ divide by 16
//**     FNOSC_LPRCDIVN        Low power Fast RC oscillator w/divide by N
//**
//**   Two-speed Oscillator Startup :
		IESO_OFF             //Disabled
//     IESO_ON              //Enabled
)

_FOSC(
//**   Clock switching and clock monitor:
//     FCKSM_CSECME &         //Both enabled
     FCKSM_CSECMD &        //Only clock switching enabled
//    FCKSM_CSDCMD &		//Both disabled
//**
//**   Single configuration for remappable I/O:
    IOL1WAY_OFF &		//Disabled
//**     IOL1WAY_ON           Enabled
//**
//**   OSC2 Pin function:
//     OSCIOFNC_ON &         //Digital I/O
    OSCIOFNC_OFF &		//OSC2 is clock O/P
//**
//**   Oscillator Selection:
//**     POSCMD_EC            External clock
     POSCMD_XT           //XT oscillator
//    POSCMD_HS            //HS oscillator
//**     POSCMD_NONE          Primary disabled
)

_FBS(
//**   Boot Segment Data Ram:
//**     RBS_LARGE_RAM        Large Sized Boot Ram
//**     RBS_MEDIUM_RAM       Medium Sized Boot Ram
//**     RBS_SMALL_RAM        Small Sized Boot Ram
    RBS_NO_RAM &		//No Boot Ram
//**
//**   Boot Segment Program Memory:
//**     BSS__IGH_LARGE_BOOT_CODE High Security Lar Boot Flash
//**     BSS_LARGE_FLASH__IGH  High Security Lar Boot Flash
//**     BSS__IGH_MEDIUM_BOOT_CODE High Security Med Boot Flash
//**     BSS_MEDIUM_FLASH__IGH High Security Med Boot Flash
//**     BSS__IGH_SMALL_BOOT_CODE High Security Small Boot Flash
//**     BSS_SMALL_FLASH__IGH  High Security Small Boot Flash
//**     BSS_LARGE_FLASH_STD   Standard Security Lar Boot Flash
//**     BSS_STRD_LARGE_BOOT_CODE Standard Security Lar Boot Flash
//**     BSS_MEDIUM_FLASH_STD  Standard Security Med Boot Flash
//**     BSS_STRD_MEDIUM_BOOT_CODE Standard Security Med Boot Flash
//**     BSS_SMALL_FLASH_STD  Standard Security Small Boot Flash
//**     BSS_STRD_SMALL_BOOT_CODEStandard Security Small Boot Flash
    BSS_NO_BOOT_CODE &	//No Boot Segment Program Memory
//**     BSS_NO_FLASH         No Boot Segment Program Memory
//**
//**    Write Protect :
//**     BWRP_WRPROTECT_ON     Enabled
    BWRP_WRPROTECT_OFF    //Disabled
)

_FSS(
//**   Secure Segment Data Ram:
//**     RSS_LARGE_RAM         Large Sized Secure Ram
//**     RSS_MEDIUM_RAM        Medium Sized Secure Ram
//**     RSS_SMALL_RAM         Small Sized Secure Ram
    RSS_NO_RAM &		//No Secure Ram
//**
//**   Secure Segment Program Memory:
//**     SSS_LARGE_FLASH__IGH  High Security Lar Secure Flash
//**     SSS_MEDIUM_FLASH__IGH High Security Med Secure Flash
//**     SSS_SMALL_FLASH__IGH  High Security Small Secure Flash
//**     SSS_LARGE_FLASH_STD   Standard Security Large Secure Flash
//**     SSS_MEDIUM_FLASH_STD  Standard Security Med Secure Flash
//**     SSS_SMALL_FLASH_STD  Standard Security Small Secure Flash
    SSS_NO_FLASH &		//No Secure Segment
//**
//**    Write Protect :
//**     SWRP_WRPROTECT_ON     Enabled
    SWRP_WRPROTECT_OFF    //Disabled
)

_FGS(
//**   Code Protect:
//**     GSS__IGH              high security protect on
//**     GSS_STD               standard security code protect on
    GSS_OFF &			//code protect off
//**
//**   Code Protect:
//**     GCP_ON               Enabled
    GCP_OFF &			//Disabled
//**
//**   Write Protect:
//**     GWRP_ON              Enabled
    GWRP_OFF             //Disabled
)

_FWDT(
//**   Watchdog Timer:
    FWDTEN_OFF &		//Disabled
//**     FWDTEN_ON            Enabled
//**
//**   Windowed WDT:
//**     WINDIS_ON            Enabled
    WINDIS_OFF &		//Disabled
//**
//**   Watchdog prescaler:
//     WDTPRE_PR32 &         //1:32
    WDTPRE_PR128 &		//1:128
//**
//**   Watchdog postscaler:
//    WDTPOST_PS1          //1:1
//**     WDTPOST_PS2          1:2
//**     WDTPOST_PS4          1:4
//**     WDTPOST_PS8          1:8
//**     WDTPOST_PS16         1:16
//**     WDTPOST_PS32         1:32
//**     WDTPOST_PS64         1:64
//**     WDTPOST_PS128        1:128
//**     WDTPOST_PS256        1:256
//**     WDTPOST_PS512        1:512
//**     WDTPOST_PS1024       1:1,024
//**     WDTPOST_PS2048       1:2,048
//**     WDTPOST_PS4096       1:4,096
//**     WDTPOST_PS8192       1:8,192
//**     WDTPOST_PS16384      1:16,384
     WDTPOST_PS32768      //1:32,768
)

_FPOR(
//**   Alternate I2C pins:
//**     ALTI2C_ON            I2C mapped to ASDA1/ASCL1
    ALTI2C_OFF &		//I2C mapped to SDA1/SCL1
//**
//**   Power-on Reset Value:
//    FPWRT_PWR1           //Disabled
//**     FPWRT_PWR2           2ms
//**     FPWRT_PWR4           4ms
//**     FPWRT_PWR8           8ms
//**     FPWRT_PWR16          16ms
//**     FPWRT_PWR32          32ms
//**     FPWRT_PWR64          64ms
     FPWRT_PWR128         //128ms
)

_FICD(
//**   JTAG Enable Bit:
    JTAGEN_OFF &		//JTAG is disabled
//**     JTAGEN_ON            JTAG is enabled
//**
//**   ICD communication channel select bits:
//**     ICS_NONE             Reserved
//**     ICS_PGD3              communicate on PGC3/EMUC3 and PGD3/EMUD3
//**     ICS_PGD2             communicate on PGC2/EMUC2 and PGD2/EMUD2
    ICS_PGD1             //communicate on PGC1/EMUC1 and PGD1/EMUD1
)

#endif	//c30

#include <stdint.h>

//global defines
#define LED_PORT		LATB					//led pins on portb
#define LED_DDR			TRISB
#define LED_PIN			0xff
#define LED_DLY			100000ul

#define IO_OUT(ddr, pins)	ddr &=~(pins)
#define IO_IN(ddr, pin)		ddr |= (pins)
#define IO_SET(port,pins)	port|= (pins)
#define IO_CLR(port,pins)	port|= (pins)
#define IO_FLP(port,pins)	port^= (pins)

#define RTCC_WREN()	{	asm volatile("push w7"); \
						asm volatile("push w8"); \
						asm volatile("disi #5"); \
						asm volatile("mov #0x55, w7"); \
						asm volatile("mov w7, _NVMKEY"); \
						asm volatile("mov #0xAA, w8"); \
						asm volatile("mov w8, _NVMKEY"); \
						asm volatile("bset _RCFGCAL, #13"); \
						asm volatile("pop w8"); \
						asm volatile("pop w7"); \
					}
//do not allow any write to rtc registers - assumes the nvmkey sequence has been sent
#define RTCC_WRDIS()	do {RCFGCALbits.RTCWREN = 0;} while (RCFGCALbits.RTCWREN == 1)
#define RTCGetSec()		(RTCRead(0x00) & 0x00ff)

//global variables

//read from rtcc
//mask: 0b00->min.sec, 0b01->weekday.hour, 0b10->month.day, 0b11->..year
uint16_t RTCRead(uint16_t mask) {
	RCFGCALbits.RTCPTR=mask;
	return RTCVAL;
}

//waste sometime
void dly(uint32_t cycle) {
	while (cycle--) Nop();
}

//initialize the rtc
void RTCInit(void) {
	PMD3bits.RTCCMD = 0;						//0->enable power to rtcc
	RTCC_WREN();								//allows write to rtc registers
	RCFGCALbits.RTCEN=1;						//start the rtc
	RTCC_WRDIS();
	//enable sosc
	__builtin_disi(0x3fff);						//di();
	__builtin_write_OSCCONL(OSCCON | (1<<1));	//1->enable sosc, 0->disable sosc
	__builtin_disi(0x0000);						//ei();
}


int main(void) {
	uint32_t sec;

	IO_OUT(LED_DDR, LED_PIN);			//LED_PIN as output
	RTCInit();							//initialize the rtc
	while (1) {
		IO_FLP(LED_PORT, LED_PIN);		//flip led_pin
		//dly(LED_DLY);					//waste sometime
		sec=RTCGetSec(); while (sec==RTCGetSec());		//wait for a second to pass
	}
	return 0;
}

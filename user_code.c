//dsPIC33duino code
// - supported chips: dspic33fj32GP302/304, dspic33fj64gp202/204/802/804, dspic33fj128gp202/204/802/804
// - free running timer2 for ticks, pwm and input capture
// - details: https://github.com/dannyf00/Minimalist-16-bit-Arduino-Clone
// - only XC16 support is provided
//
// - version history
// - v0.1, 12/29/2022: initial porting from pic24duino
//
//
//               dsPIC33FJ
//              |=====================|
//    Vcc       |                     |
//     |        |                Vcap |>--[.1u]-+->GND
//     |        |                     |         |
//     +-[10K]-<| MCLR        DISVreg |>--------+
//              |                     |
//              |                     |
//     +------->| OSCI            Vdd |>--+------>Vcc
//  [Xtal]      |                     | [.1u]
//     +-------<| OSCO            Vss |>--+------>GND
//              |                     |
//              |                 RP0 |>---------->Uart2TX
//              |                     |
//              |                 RB5 |>---------->LED
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |=====================|
//
//

#include "dspic33duino.h"						//we use dspic33duino
#include "fractal.h"

//hardware configuration
#define LED			PB5							//led pin
#define LED_DLY		(F_CPU / 2)					//half a second
//end hardware configuration

//global defines

//global variables


//tune frc
//using rtc to test for ticks on the main clock
//return: number of ticks measured over a period of time specified by sec
uint32_t rtcTks(uint8_t sec) {
	uint32_t tks;
	uint32_t tmp;

	tmp = RTCGetSec(); while (RTCGetSec() == tmp) continue;	//wait for the second to change
	//now a new second has arrived
	tks=ticks();
	while (sec--) {
		tmp = RTCGetSec(); while (RTCGetSec() == tmp) continue;	//wait for the second to change
	}
	return ticks() - tks;
}


//oc1isr
void oc1ISR(void) {
	sRTCISR();									//run software rtc
}

//smoothing
uint32_t smth_exp(uint32_t dat) {
	static uint32_t sum=4*F_FRC/2;
	static uint32_t avg=0;

	//if (sum==0) sum=dat*4;
	avg=sum/4;
	sum+= (int32_t) (dat - avg)/4;
	return avg;
}

//flip led
void led_flp(void) {
	pinFlip(LED);								//flip led
}

//user defined set up code
void setup(void) {
	//SystemCoreClockFRC();						//set systemclock to frc
	//SystemCoreClockFRCPLL2x();				//set systemclock to frcpll
	SystemCoreClockPOSC();						//set systemclock to posc
	//SystemCoreClockPOSCPLL2x();				//set systemclock to poscpll

    pinMode(LED, OUTPUT);						//led as output pin

    //initialize the uart
    //uart1Init(UART_BR9600);					//initialize uart1
    uart2Init(UART_BR38K4);						//initialize uart2

	//rtcc initialization: -1 indicates that the RTC is uninitiated
	RTCInit();  if (RTC2time(NULL) == -1)  time2RTC(1234567890ul);	//reset RTC only if the data is invalid. epoch time: Friday, February 13, 2009 11:31:30 PM
	//RTCSetCal(+10);							//optional: calibrate the RTC

	//initialize software rtc: -1 indicates that the sRTC is uninitiated
	sRTCInit(); time2sRTC(RTC2time(NULL));		//reset sRTC using the hardware RTC
	//sRTCSetCal(+10);							//optional: calibrate the software RTC

	//option 1: using output compare isr to update the sRTC
	//oc1Init(F_CPU / sRTC_CALLRATE);				//initialize oc1 to run at a rate of sRTC_CALLRATE
	//oc1AttachISR(oc1ISR);						//install oc1isr

    //enable interrupts
    ei();
}


//user defined main loop
void loop(void) {
    static uint32_t tick0=0;
    uint32_t tmp0;
    time_t time_tmp;
    static char halfsec, shalfsec;
    //static uint32_t last=0;					//millis for current update and last update
    //static uint32_t last_sRTC=0;				//last rtc invocatin, in millis
    //long i;

	sRTCUpdate();								//update software rtc
	//digitalWrite(LED, RTCHalfsec());
	//if ( halfsec!= RTCHalfsec()) {pinFlip(LED);  halfsec= RTCHalfsec();}
	//if (shalfsec!=sRTCHalfsec()) {pinFlip(LED); shalfsec=sRTCHalfsec();}

    //if enough time has elapsed
    if (ticks() - tick0 > LED_DLY) {			//if enough time has passed
        tick0 += LED_DLY;						//advance to the next match point
        pinFlip(LED);							//digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks

        //measure timing
        tmp0=ticks();
        //put some tasks here
        //mandelbrot();							//fractal benchmark
        //for (i=0; i<10000; i++) pinFlip(LED);	//520k/int, 550k/long
        tmp0=ticks() - tmp0;

        //display something
        u2Print("F_CPU =                    ", F_CPU);
        u2Print("ticks =                    ", ticks());
        //u2Print("tmp0  =                    ", tmp0);
        //u2Print("u2bps =                    ", u2bps());
        //u2Print("RCFGCA=                    ", RCFGCAL);
        //u2Print("RTChsc=                    ", sRTCHalfsec());
        //u2Print("d_time=                    ", RTC2time(NULL) - sRTC2time(NULL));
        //u2Print(" RTCtm=                    ",  RTC2time(NULL));
        //u2Print("sRTCtm=                    ", sRTC2time(NULL));
        u2Print("hRTCtm=                    ",  RTC2time(&time_tmp)); uart2Puts(ctime(&time_tmp));
        //uart2Puts(ctime(&time_tmp));
        //u2Print("rtcTks=                    ", smth_exp(rtcTks(4)/4)*2);
        //u2Print("sRTCtm=                    ", sRTC2time(&time_tmp)); uart2Puts(ctime(&time_tmp));
        u2Println();
        //time2sRTC(time_tmp + 3600ul*24*3);		//advance time: 3600sec/hr*24hr*days
    }
}

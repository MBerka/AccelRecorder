/*
 * AccelRecorderProgram.c
 * 26bytes/meas*300meas/s*3600s/h*48h
 *
 * Created: 2014-07-04 12:00:00
 * Author: Martin Berka, file access process reused from code by Marek Zylinski
 * When SD card is inserted, activates accelerometer and  records 300 
 * measurements/s from 3 ADC channels to SD Card while flashing diode.
 * Otherwise, enters low-power mode.
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "xitoa.h"
#include "ff.h"
#include "diskio.h"

#include "avr_compiler.h"
#include "adc_driver.h"
#include "clksys_driver.h"
// #include "rtc32_driver.h" //Real-time clock not in use.
#include "wdt.h"

#define P_DIODE PIN7_bm //Pin for the communication diode — MSB, 128
#define LEDPORT PORTC //Port with the diode. Pins 0-6 are empty
#define AD_XYZ_PORT PORTA //Port with the accelerometer axes' voltages
#define P_XYZ 224 //Accelerometer data pins: 5, 6, 7.
#define AD_AUX_PORT PORTB //Port for steering the accelerometer, 0g detection; ADC/DAC
#define P_AUX_CONTROL 112 //Steering with pins 4,5,6: 
//gravity select (1.5g vs 6g), self-test (off, on), sleep (on, off): power saving=clear all
#define ACC_ACTIVE 80 //Pins 4 and 6: when high, sleep off, 6g sensitivity

volatile uint8_t file_num  = 1;
const uint8_t seconds_before_flashing = 4;

volatile unsigned char results_file[15] = {"0\0"};
volatile unsigned char list_file[15] = {"0\0"};
	
//Key boolean variables
volatile uint8_t recording = 0;
volatile uint8_t measuring = 0;

volatile uint8_t time = 0;//Given time of exercise
volatile uint8_t state = 0;//Specifies diodes to light
volatile uint8_t time_counter = 0;

//ADC variables
volatile uint8_t results_flag = 0;
volatile uint16_t result[3];
uint8_t channels[8] = {ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXPOS_PIN1_gc,ADC_CH_MUXPOS_PIN2_gc,ADC_CH_MUXPOS_PIN3_gc,ADC_CH_MUXPOS_PIN4_gc,ADC_CH_MUXPOS_PIN5_gc,ADC_CH_MUXPOS_PIN6_gc,ADC_CH_MUXPOS_PIN7_gc};
uint8_t xyz_channel_numbers[3] = {5, 6, 7}; //Ostatnie 3 piny

volatile unsigned char str[44]={"0\0"};

//FatFs variables	
	FATFS fs;          // Work area (file system object) for the volume
	FIL fsrc;      /* file objects */
	FIL fzap;
	FIL flis;      /* file objects */
	FRESULT res;       // Petit FatFs function common result code
    char buff[120];     // File read buffer
    UINT br;           // File read count

void sleep_mode()
{
	sei();
	/* Choose sleep mode (power down, asynchronous I/O wakeup) */
	SLEEP.CTRL = SLEEP_SMODE_PDOWN_gc; 

	/* Disable timers on ports except D, which is needed to wake, and already-off E and F */
	PR.PRGEN  = 0x1F;
	PR.PRPA   = 0x7F;
	PR.PRPB   = 0x7F;
	PR.PRPC   = 0x7F;

	_delay_ms(6000.0);
	CCPWrite(&MCU.MCUCR, 0x1);
// 	PORTB.DIR = 0xFF;
// 	PORTB.OUT = 0;

	SLEEP.CTRL |= SLEEP_SEN_bm;
	asm volatile ("sleep");
	for(;;);
}

void get_file_name ()
{
	volatile FILINFO fno;
	file_num = 0;
	results_file[12] = '\0'; //"0\0" TYTUL PLIKU Z DANYMI DO LISTY
	do
	{
		sprintf((char *)results_file, "ACCEL%03d.TXT",file_num); //Create results file with new number
		res = f_stat((const TCHAR*)results_file, &fno); //See if file exists
//  		if(res == 0)
//  		{
// 			f_open(&fzap,  results_file, FA_READ);
//  			if(f_size(&fzap)<1000)
//  			{
//  				res = 1; //Overwrite empty files
//  			}
// 			f_close(&fzap);
//  		}
		file_num++;
	} while ((res == 0)&&(file_num<1000)); //Keep looking until new file found or too many files exist
}

char KH(int numer)
{
	return (char)(numer/100+28);
}
char KL(int numer)
{
	return (char)(numer%100+28);
}

int main(void)
{
	wdt_disable();
	PORTD_DIRCLR = PIN3_bm; //Input pin (detect SD card)
	PORTD.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTD.INT0MASK = PIN3_bm;
//	PORTD.INTCTRL = PORT_INT0LVL_LO_gc;
	PORTD.INTCTRL = 0x0f;
	
//	PMIC.CTRL |= PMIC_LOLVLEN_bm;
/*	PORTS.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
    PORTC.INT0MASK = PIN0_bm;
    PORTC.INTCTRL = PORT_INT0LVL_LO_gc;*/
	
	PR.PRPF   = 0x7F; //Nothing is connected to this port
	/** Disable the unused oscillators: 32 MHz, internal 32 kHz, external */
	OSC.CTRL &= ~(OSC_RC32MEN_bm | OSC_RC32KEN_bm);
	PORTF.DIR = 0;
	PORTF.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN1CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTF.PIN7CTRL = PORT_OPC_PULLUP_gc;
	PORTE.DIR = 0; //Port E contains lines to an unused digital accelerometer
	//Modify if it is connected
	PORTE.PIN2CTRL = PORT_OPC_PULLUP_gc;	
	PORTE.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTE.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTE.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTE.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTE.PIN7CTRL = PORT_OPC_PULLUP_gc;	
	PR.PRPE   = 0x7F;
	
	recording = 0;
	LEDPORT.OUTTGL = P_DIODE;//Quick indication of startup

	//Ustawienie zegara na 16 MHz
  	CLKSYS_PLL_Config( OSC_PLLSRC_RC2M_gc, 8 );
	CLKSYS_Enable( OSC_PLLEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
	do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
	CLKSYS_Disable( OSC_XOSCEN_bm );
	
	//ADC calibration
	ADC_CalibrationValues_Load(&ADCA);
	ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Unsigned, ADC_RESOLUTION_12BIT_gc);
	ADC_Reference_Config(&ADCA, ADC_REFSEL_AREFA_gc);
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV8_gc);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_SINGLEENDED_gc,
	                                 ADC_CH_GAIN_1X_gc);							 
	ADC_Enable(&ADCA);
	ADC_Wait_8MHz(&ADCA);
	
	LEDPORT.DIR = P_DIODE; //Output to diode pin
	LEDPORT.OUT = P_DIODE; //Diode on
	
	AD_AUX_PORT.DIR = P_AUX_CONTROL; //Pins 4-6 control accelerometer
	AD_AUX_PORT.OUTCLR = P_AUX_CONTROL; //Asleep, self-test off, 1.5g sensitivity (all low)
	
	AD_XYZ_PORT.DIR = 0; //No outputs, some pins disconnected
			
	//Timer settings for ADC (successive measurements)
	TCD1.PERL = 50;             // 200 interrupts per second was 55L, 39H
	TCD1.PERH = 25;
    TCD1.CTRLA = 4;             // Prescaler 8
    TCD1.INTCTRLA = 2; 
	
	//Timer for measuring seconds
	TCC1.PERL = 25;                 // Set period 10000
	TCC1.PERH = 245;
	TCC1.CTRLA = 6;                // Prescaler  16
	TCC1.INTCTRLA = 3;                // Enable overflow interrupt
	PMIC_CTRL = 7; //PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	                   // Enable medium level interrupts
		
	while(1)
	{
		if((PORTD.IN & _BV(3))) //Lowers power consumption when SD card removed; interrupt restores
		{
			sleep_mode();
		}
//		_delay_ms(100); //Czeka az bedzie karta - wstawienie tu prostego warunku nie dziala
		
		AD_AUX_PORT.OUTSET = ACC_ACTIVE; //Wake and set 6g
		
		if(recording==0) //Not currently recording; begin
		{
			time_counter = seconds_before_flashing;
			ENTER_CRITICAL_REGION(); //No interruptions
			res = 1;
			while (res != 0)
			{
				res = f_mount(0,&fs); //File system: mount SD card
			}
				
			//When ready, start file
			get_file_name();//Get name of file to write to
			//Open file for writing
			do 
			{
				res = f_open(&fzap,  results_file, FA_OPEN_ALWAYS | FA_WRITE);
			} while (res!=0);
			res = f_lseek(&fzap, f_size(&fzap)); //Jump to end
			
			sprintf((char *)str, "%c%c%c%c%c%c\n", KH(9898), KL(9898), KH(9898), KL(9898), KH(9898), KL(9898));
			//sprintf((char *)str, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", KH(9999), KL(9999), KH(9999), KL(9999),KH(9999),KL(9999),KH(9999), KL(9999), KH(9999), KL(9999),KH(9999),KL(9999),KH(9999), KL(9999),KH(9999), KL(9999));
			f_puts((const TCHAR*)str,&fzap);
			
			LEAVE_CRITICAL_REGION(); //Writing safe now
			
			recording = 1;
			LEDPORT.OUTCLR = P_DIODE;//Diode on
		}
	
		sei(); //Interrupts on
		while(recording) //Recording loop, active throughout most of program execution
		{			
			if (results_flag)
			{	
				ENTER_CRITICAL_REGION();			
				f_puts(str,&fzap); //Write data produced in interrupt to file
				LEAVE_CRITICAL_REGION();
				results_flag = 0; //Do nothing until next interrupt
			}
		}
		cli(); //Interrupts on
	}
}

ISR(TCC1_OVF_vect) 
{ //Interrupt for measuring time (every second)
	if (recording) //When recording is active
	{
		LEDPORT.OUTSET = P_DIODE;
		//Periodic checking of conditions for further recording
		if((!measuring)||(PORTD.IN & _BV(3))) //If measurement data or card absent
		{
			recording = 0; //Halt
			AD_AUX_PORT.OUTCLR = P_AUX_CONTROL; //Accelerometer: sleep; SL, ST, GS low
			return;
		}
	
		measuring = 0; //Should be set to true again before next such interrupt
		
		if(time_counter > 0)
		{ 
			time_counter--; //countdown to next, longer time interval
		}
		
		//Insert time marker every second
		
		sprintf((char *)str, "%c%c%c%c%c%c\n", KH(9999), KL(9999), KH(9999), KL(9999), KH(9999), KL(9999));
		//sprintf((char *)str, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", KH(9999), KL(9999), KH(9999), KL(9999),KH(9999),KL(9999),KH(9999), KL(9999), KH(9999), KL(9999),KH(9999),KL(9999),KH(9999), KL(9999),KH(9999), KL(9999));
		ENTER_CRITICAL_REGION();
		f_puts((const TCHAR*)str,&fzap);
		if(time_counter == 0)
		{
			time_counter = seconds_before_flashing; 
			//Make recording permanent every 5 seconds
			f_sync(&fzap); //The last line in the file is always a time marker
			LEDPORT.OUTCLR = P_DIODE;  //Flashing to indicate that device is working
		}
		LEAVE_CRITICAL_REGION();
	}
}

ISR(TCD1_OVF_vect) 
{ //Performs ADC measurements and writes them to the file
	if (recording)
	{
		for (uint8_t i = 0; i<3; i++)
		{
			//ADC_Ch_InputMux_Config(&ADCA.CH0, channels[xyz_channel_numbers[i]], ADC_CH_MUXNEG_PIN0_gc);//Wybor kanalu: pretwarzacz A, pin +, pin -
			ADC_Ch_InputMux_Config(&ADCA.CH0, ((xyz_channel_numbers[i])<<3), ADC_CH_MUXNEG_PIN0_gc);//Wybor kanalu: pretwarzacz A, pin +, pin -
     		ADC_Ch_Conversion_Start(&ADCA.CH0);
			while(!ADC_Ch_Conversion_Complete(&ADCA.CH0));
			result[i] = ADC_ResultCh_GetWord(&ADCA.CH0);
		}

		measuring = 1;
		results_flag = 1;
		
		//printf("%c%c", (char)(wyniki[licznik]/100+skok), (char)(wyniki[licznik]%100+skok)); Podstawa kodowania uzytego na nastepnej linii
		sprintf((char *)str, "%c%c%c%c%c%c\n", KH(result[0]), KL(result[0]), KH(result[1]), KL(result[1]),KH(result[2]),KL(result[2]));
		//sprintf((char *)str, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", KH(result[0]), KL(result[0]), KH(result[1]), KL(result[1]),KH(result[2]),KL(result[2]), KH(result[3]), KL(result[3]), KH(result[4]), KL(result[4]),KH(result[5]),KL(result[5]), KH(result[6]), KL(result[6]), KH(result[7]), KL(result[7]));
	}
}

ISR(PORTD_INT0_vect)
{
    // Quick visual confirmation of startup
    LEDPORT.OUTTGL = P_DIODE;

//	CCPWrite( &RST.CTRL, RST_SWRST_bm ); //Inadequate
	//wdt_enable(WDTO_15MS); //Fully reset as soon as possible
	wdt_enable();
    while(1){};
}
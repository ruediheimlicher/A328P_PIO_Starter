//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//

#include <Arduino.h> 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include "lcd.c"


#include "display.h"

#include "text.h"



#define SHOWDISPLAY 0


//#include "adc.c"
#define TW_STATUS   (TWSR & TW_STATUS_MASK)



//***********************************
						
uint8_t  LOK_ADRESSE = 0xCC; //	11001100	Trinär
//									
//***********************************

/*
 commands
 LO     0x0202  // 0000001000000010
 OPEN   0x02FE  // 0000001011111110
 HI     0xFEFE  // 1111111011111110
 */


//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
#define INPIN   PIND  // Input signal

#define DATAPIN  2 


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD
#define LOOPLED			5

#define INT0_RISING	   0
#define INT0_FALLING		1


volatile uint8_t   loopstatus=0x00;            


void lcd_puts(const char *s);

// EADOGM
volatile uint16_t laufsekunde=0;
volatile uint8_t laufminute=0;
volatile uint8_t laufstunde=0;

volatile uint16_t motorsekunde=0;

volatile uint16_t motorminute=0;

volatile uint16_t stopsekunde=0;

volatile uint16_t stopminute=0;
volatile uint16_t batteriespannung =0;
volatile uint16_t updatecounter =0;

volatile uint8_t                 curr_screen=0; // aktueller screen
volatile uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
volatile uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
volatile uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
volatile uint8_t                 last_cursorspalte=0; // letzte colonne des cursors


volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
volatile uint16_t                 blink_cursorpos=0xFFFF;

volatile uint8_t                 displaystatus = 0; // bits for sending display data
                                          // 0-3: sendposition
                                          // 4,5,6: 
                                          // 7: aktivbit


volatile uint8_t                 displaydata[8]  = {};



uint16_t spicounter=0;

//

volatile uint8_t	INT0status=0x00;				
volatile uint8_t	signalstatus=0x00; // status TRIT
volatile uint8_t  pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   



volatile uint16_t MEMBuffer = 0;

//volatile uint16_t	HIimpulsdauer=0;			//	Dauer des LOimpulsdaueres Definitiv

volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;
//volatile uint16_t   newlokdata = 0;

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;

// ***
volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;

volatile uint8_t     oldspeed = 0;
volatile uint8_t     newspeed = 0;
volatile uint8_t     startspeed = 0; // Anlaufimpuls
volatile uint8_t     speedcode = 0;
volatile int8_t      speedintervall = 0;

volatile uint8_t   dimmcounter = 0; // LED dimmwertcounter
volatile uint8_t   ledpwm = 0x40; // LED PWM 50%
volatile uint8_t   ledstatus=0; // status LED


volatile uint8_t   ledonpin = LAMPEA_PIN; // Stirnlampe ON
volatile uint8_t   ledoffpin = LAMPEB_PIN; // Stirnlampe OFF

// ***
volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t     pwmpin = MOTORA_PIN;           // Motor PWM
volatile uint8_t     richtungpin = MOTORB_PIN;      // Motor Richtung


volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;




volatile uint8_t   taskcounter = 0;

volatile uint8_t   speedlookup[15] = {};
uint8_t speedlookuptable[10][15] =
{
   {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252},
   {0,30,40,50,60,70,80,90,100,110,120,130,140,150,160},
   {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140},
   {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100},
   {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80},
   
   //{0,41,42,44,47,51,56,61,67,74,82,90,99,109,120},
   {0,11,15,19,26,34,43,55,68,82,98,116,136,157,180},
   {0,41,43,45,49,54,60,66,74,82,92,103,114,127,140},
   {0,41,44,48,53,59,67,77,87,99,113,128,144,161,180},
   {0,42,45,50,57,65,75,87,101,116,134,153,173,196,220},
   {0,42,45,51,58,68,79,93,108,125,144,165,188,213,240}
};


volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0xEF;
uint16_t speedchangetakt = 0x350; // takt fuer beschleunigen/bremsen


volatile uint8_t loktyptable[4];

volatile uint8_t speedindex = 5;

volatile uint8_t   maxspeed =  252;//prov.
volatile uint8_t   minspeed =  0;
uint16_t displaycounter0;
uint16_t displaycounter1;

uint16_t displayfenstercounter = 0; // counter fuer abgelaufene Zeit im Display-Fenster

//extern uint8_t display_init(void);//




void spi_init(void) // SPI-Pins aktivieren
{
   // https://embedds.com/serial-peripheral-interface-in-avr-microcontrollers/
   //set MOSI, SCK and SS as output
   DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);
   //set SS to high
   PORTB |= (1<<PB2);
   //enable master SPI at clock rate Fck/2
   SPCR = (1<<SPE)|(1<<MSTR);
   SPSR |= (1<<SPI2X);
}


void slaveinit(void)
{
   
   LCD_DDR |= (1<<LCD_RSDS_PIN);
   LCD_DDR |= (1<<LCD_CLOCK_PIN);
   LCD_DDR |= (1<<LCD_RSDS_PIN);

   

   
	//LOOPLEDPORT |=(1<<LOOPLED);
   LOOPLEDDDR |= (1<<LOOPLED);
   
  

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

   
    
   

   //initADC(MEM);
   
   // default
   //LOOPLEDPORT |=(1<<LOOPLED);
   pwmpin = MOTORA_PIN;
   richtungpin = MOTORB_PIN;
   ledonpin = LAMPEA_PIN;
   ledoffpin = LAMPEB_PIN;
   
  
   maxspeed = speedlookup[14];
   minspeed = speedlookup[1];
   
   loopstatus |= (1<<FIRSTRUNBIT);

   // TWI
   DDRC |= (1<<5);   //Pin 0 von PORT C als Ausgang (SCL)
   PORTC |= (1<<5);   //   ON
   DDRC |= (1<<4);   //Pin 1 von PORT C als Ausgang (SDA)
   PORTC |= (1<<4);   //   ON

   

   
   //LOOPLEDPORT |=(1<<LOOPLED);
}


void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
//	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   TCCR2B |= (1<<CS21); // no prescaler
	//OC2 akt


	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
   TIMSK2 |=(1<<TOIE2);        //interrupt on Compare Match A
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf HI-impulsdauer
} 



void int0_init(void)
{
   EICRA |= (1 << ISC00) | (1 << ISC01);  // Trigger interrupt on rising edge
   EIMSK |= (1 << INT0);  // Enable external interrupt INT0

 //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   INT0status |= (1<<INT0_WAIT);
}




int main (void) 
{
   
   
	slaveinit();
   
 //  int0_init();
	
	// initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	_delay_ms(800);
	lcd_cls();
	lcd_puts("A328_PIO_Start");
	
   
   
	//uint16_t loopcount0=0;
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   uint16_t loopcount2=0;

	
	_delay_ms(200);

   

   
   sei();
   

    
  
  
	while (1)
   {  
      //OSZI_B_LO();
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      

      
         
         loopcount0++;
         if (loopcount0>=refreshtakt)
         {
            //OSZI_B_LO();
            //OSZIATOG;
            //LOOPLEDPORT ^= (1<<LOOPLED); 
            
            loopcount0=0;
            loopcount1++;
            if (loopcount1>=refreshtakt)
            {
               loopcount1 = 0;
               LOOPLEDPORT ^= (1<<LOOPLED); 
               lcd_gotoxy(0,1);
               lcd_putint(loopcount2);
               loopcount2++;
            }
            
            
            loopcount0=0;
            

            
            
            
            
            
            //OSZI_B_HI();
         }  // loopcount0>=refreshtakt
         OSZI_B_HI();
         
      
   }//while


 return 0;
}

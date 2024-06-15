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


//#include <Wire.h> // ok
//#include <U8g2lib.h>
//#include <U8x8lib.h>

// U8X8_SSD1327_WS_128X128_HW_I2C([reset [, clock, data]])

//U8X8_SSD1327_WS_128X128_HW_I2C u8x8(A4,A5); // ok
// https://github.com/olikraus/u8g2/wiki/u8g2setupc#ssd1327-ws_128x128


// I2C address of the SSD1327 OLED display (assuming default 0x3D)
#define SSD1327_ADDRESS 0x3D


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD
#define LOOPLED			5

#define INT0_RISING	   0
#define INT0_FALLING		1


volatile uint8_t   loopstatus=0x00;            


void lcd_puts(const char *s);

uint16_t spicounter=0;

//

volatile uint8_t	INT0status=0x00;				
volatile uint8_t	signalstatus=0x00; // status TRIT
volatile uint8_t  pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   





volatile uint8_t   taskcounter = 0;



uint16_t refreshtakt = 0x1FF;
volatile uint8_t loktyptable[4];

volatile uint8_t speedindex = 5;

volatile uint8_t   maxspeed =  252;//prov.
volatile uint8_t   minspeed =  0;
uint16_t displaycounter0;
uint16_t displaycounter1;

uint16_t displayfenstercounter = 0; // counter fuer abgelaufene Zeit im Display-Fenster

//extern uint8_t display_init(void);//

// i2c
void i2c_init(void) {
    // Set SCL frequency to 100kHz with a 16MHz system clock
    TWSR = 0x00;
    TWBR = 0x48;  // (16MHz / 100kHz - 16) / 2 = 72 = 0x48
    TWCR = (1 << TWEN);  // Enable TWI
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}
void oled_command(uint8_t command) {
    i2c_start();
    i2c_write((SSD1327_ADDRESS << 1) | 0); // 0 for write
    i2c_write(0x80); // Co = 1, D/C# = 0
    i2c_write(command);
    i2c_stop();
}

void oled_data(uint8_t data) {
    i2c_start();
    i2c_write((SSD1327_ADDRESS << 1) | 0); // 0 for write
    i2c_write(0x40); // Co = 0, D/C# = 1
    i2c_write(data);
    i2c_stop();
}
void oled_init(void) {
    // Example initialization sequence (refer to SSD1327 datasheet)
    oled_command(0xAE); // Display off
    oled_command(0xA8); // Set multiplex ratio
    oled_command(0x7F); // 128MUX
    oled_command(0xA1); // Set display start line
    oled_command(0x00); // Start line 0
    oled_command(0xA2); // Set display offset
    oled_command(0x00); // Offset 0
    oled_command(0xA4); // Normal display
    oled_command(0xA6); // Set normal display mode
    oled_command(0xAF); // Display on
}


// end i2c


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
   //pwmpin = MOTORA_PIN;
   //richtungpin = MOTORB_PIN;
   //ledonpin = LAMPEA_PIN;
   //ledoffpin = LAMPEB_PIN;
   
  
   //maxspeed = speedlookup[14];
  // minspeed = speedlookup[1];
   
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
	_delay_ms(2);
   i2c_init();
_delay_ms(2);
   oled_init();
   _delay_ms(2);
	//uint16_t loopcount0=0;
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   uint16_t loopcount2=0;

	
	_delay_ms(2);
   //u8x8.begin(); // blocks loop
   _delay_ms(2);

   
   sei();

	while (1)
   {  
      //OSZI_B_LO();
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      
      
         
         loopcount0++;
         if (loopcount0>=2*refreshtakt)
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

               // OLED
               SYNC_LO();
         
               oled_command(0x21); // Column address
               oled_command(0x00); // Column start address
               oled_command(0x7F); // Column end address
               oled_command(0x22); // Page address
               oled_command(0x00); // Page start address
               oled_command(0x07); // Page end address

               for (uint16_t i = 0; i < 128 * 8; i++) 
               {
                  oled_data(0xFF); // Fill the screen with pattern
               }
               SYNC_HI();
               if (loopcount2 > 10)
               {
                  loopcount2 = 0;
                for (uint16_t i = 0; i < 128 * 8; i++) 
               {
                     oled_data(0x00); // Clear the screen
               }
                 
               }
               

            }
            
            
            loopcount0=0;
            

            
            
            
            
            
            //OSZI_B_HI();
         }  // loopcount0>=refreshtakt
         OSZI_B_HI();
         
      
   }//while


 return 0;
}

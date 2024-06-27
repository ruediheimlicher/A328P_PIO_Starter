#include <Arduino.h> 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include "defines.h"
#include <stdint.h>

#include "lcd.c" // LCD-Stuff

#include <U8g2lib.h>
#include <U8x8lib.h>
//#include <MUIU8g2.h>

#include <Wire.h>
#define WIRE Wire

//U8X8_SSD1327_WS_128X128_HW_I2C u8x8(A4,A5); // ok
//U8X8_SSD1327_WS_128X128_SW_I2C u8x8(A4, A5 , PC0);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8x8(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // ok
// https://github.com/olikraus/u8g2/wiki/u8g2setupc#ssd1327-ws_128x128

#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD
#define LOOPLED			0

volatile uint8_t   loopstatus=0x00;            

uint16_t loopcount0=0;
uint16_t loopcount1=0;
uint16_t loopcount2=0;

uint16_t refreshtakt = 0x1FFF;

void slaveinit(void)
{   
   LOOPLEDDDR |= (1<<LOOPLED);

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

}  

    
int main (void) 
{
	slaveinit();  
 	// initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("A328_PIO_Start");
   while (!Serial)
     delay(10);
  Serial.println("\nI2C Scanner");
  
	
	_delay_ms(200);
   u8x8.begin(); // blocks loop
   _delay_ms(200);
   
   sei();
	while (1)
   {  
         loopcount0++;
         if (loopcount0>=refreshtakt)
         {
            loopcount0=0;
            loopcount1++;
            if (loopcount1>=2*refreshtakt)
            {
               loopcount1 = 0;
               LOOPLEDPORT ^= (1<<LOOPLED); 
               lcd_gotoxy(0,1);
               lcd_putint(loopcount2);
               loopcount2++;
            }           
            loopcount0=0;
         }  // loopcount0>=refreshtakt
   }//while
 return 0;
}

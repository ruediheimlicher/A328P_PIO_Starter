//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//

#include <Arduino.h> 
#include <Wire.h>

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

#include <Adafruit_SSD1327.h>
#include <Adafruit_GFX.h>
#include <splash.h>

//#include <Wire.h> // ok
//#include <U8g2lib.h>
//#include <U8x8lib.h>

// U8X8_SSD1327_WS_128X128_HW_I2C([reset [, clock, data]])

//U8X8_SSD1327_WS_128X128_HW_I2C u8x8(A4,A5); // ok
// https://github.com/olikraus/u8g2/wiki/u8g2setupc#ssd1327-ws_128x128


// I2C address of the SSD1327 OLED display (assuming default 0x3D)
#define SSD1327_ADDRESS 0x3D


// Used for software SPI
#define OLED_CLK 13
#define OLED_MOSI 11

// Used for software or hardware SPI
#define OLED_CS 10
#define OLED_DC 8




// Used for I2C or SPI
#define OLED_RESET -1
// I2C
//Adafruit_SSD1327 display(128, 128, &Wire, OLED_RESET, 1000000);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };





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



uint16_t refreshtakt = 0xFF;
volatile uint8_t loktyptable[4];

volatile uint8_t speedindex = 5;

volatile uint8_t   maxspeed =  252;//prov.
volatile uint8_t   minspeed =  0;
uint16_t displaycounter0;
uint16_t displaycounter1;

uint16_t displayfenstercounter = 0; // counter fuer abgelaufene Zeit im Display-Fenster

//extern uint8_t display_init(void);//

// i2c

#define SCL_CLOCK  100000L

void i2c_init(void) {
    // Set SCL frequency to 100kHz with a 16MHz system clock
    TWSR = 0x00;
    TWBR = 0x24;  // (16MHz / 100kHz - 16) / 2 = 72 = 0x48
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

// Basic 5x7 font
const uint8_t font5x7[][5] PROGMEM = {
    // Add character patterns here (for simplicity, not all characters are included)
    { 0x00, 0x00, 0x00, 0x00, 0x00,}, // (space)
{ 0x00, 0x00, 0x5F, 0x00, 0x00,}, // !
{ 0x00, 0x07, 0x00, 0x07, 0x00,}, // "
{ 0x14, 0x7F, 0x14, 0x7F, 0x14,}, // #
{ 0x24, 0x2A, 0x7F, 0x2A, 0x12,}, // $
{ 0x23, 0x13, 0x08, 0x64, 0x62,}, // %
{ 0x36, 0x49, 0x55, 0x22, 0x50,}, // &
{ 0x00, 0x05, 0x03, 0x00, 0x00,}, // '
{ 0x00, 0x1C, 0x22, 0x41, 0x00,}, // (
{ 0x00, 0x41, 0x22, 0x1C, 0x00,}, // )
{ 0x08, 0x2A, 0x1C, 0x2A, 0x08,}, // *
{ 0x08, 0x08, 0x3E, 0x08, 0x08,}, // +
{ 0x00, 0x50, 0x30, 0x00, 0x00,}, // ,
{ 0x08, 0x08, 0x08, 0x08, 0x08,}, // -
{ 0x00, 0x60, 0x60, 0x00, 0x00,}, // .
{ 0x20, 0x10, 0x08, 0x04, 0x02,}, // /
{ 0x3E, 0x51, 0x49, 0x45, 0x3E,}, // 0
{ 0x00, 0x42, 0x7F, 0x40, 0x00,}, // 1
{ 0x42, 0x61, 0x51, 0x49, 0x46,}, // 2
{ 0x21, 0x41, 0x45, 0x4B, 0x31,}, // 3
{ 0x18, 0x14, 0x12, 0x7F, 0x10,}, // 4
{ 0x27, 0x45, 0x45, 0x45, 0x39,}, // 5
{ 0x3C, 0x4A, 0x49, 0x49, 0x30,}, // 6
{ 0x01, 0x71, 0x09, 0x05, 0x03,}, // 7
{ 0x36, 0x49, 0x49, 0x49, 0x36,}, // 8
{ 0x06, 0x49, 0x49, 0x29, 0x1E,}, // 9
{ 0x00, 0x36, 0x36, 0x00, 0x00,}, // :
{ 0x00, 0x56, 0x36, 0x00, 0x00,}, // ;
{ 0x00, 0x08, 0x14, 0x22, 0x41,}, // <
{ 0x14, 0x14, 0x14, 0x14, 0x14,}, // =
{ 0x41, 0x22, 0x14, 0x08, 0x00,}, // >
{ 0x02, 0x01, 0x51, 0x09, 0x06,}, // ?
{ 0x32, 0x49, 0x79, 0x41, 0x3E,}, // @
{ 0x7E, 0x11, 0x11, 0x11, 0x7E,}, // A
{ 0x7F, 0x49, 0x49, 0x49, 0x36,}, // B
{ 0x3E, 0x41, 0x41, 0x41, 0x22,}, // C
{ 0x7F, 0x41, 0x41, 0x22, 0x1C,}, // D
{ 0x7F, 0x49, 0x49, 0x49, 0x41,}, // E
{ 0x7F, 0x09, 0x09, 0x01, 0x01,}, // F
{ 0x3E, 0x41, 0x41, 0x51, 0x32,}, // G
{ 0x7F, 0x08, 0x08, 0x08, 0x7F,}, // H
{ 0x00, 0x41, 0x7F, 0x41, 0x00,}, // I
{ 0x20, 0x40, 0x41, 0x3F, 0x01,}, // J
{ 0x7F, 0x08, 0x14, 0x22, 0x41,}, // K
{ 0x7F, 0x40, 0x40, 0x40, 0x40,}, // L
{ 0x7F, 0x02, 0x04, 0x02, 0x7F,}, // M
{ 0x7F, 0x04, 0x08, 0x10, 0x7F,}, // N
{ 0x3E, 0x41, 0x41, 0x41, 0x3E,}, // O
{ 0x7F, 0x09, 0x09, 0x09, 0x06,}, // P
{ 0x3E, 0x41, 0x51, 0x21, 0x5E,}, // Q
{ 0x7F, 0x09, 0x19, 0x29, 0x46,}, // R
{ 0x46, 0x49, 0x49, 0x49, 0x31,}, // S
{ 0x01, 0x01, 0x7F, 0x01, 0x01,}, // T
{ 0x3F, 0x40, 0x40, 0x40, 0x3F,}, // U
{ 0x1F, 0x20, 0x40, 0x20, 0x1F,}, // V
{ 0x7F, 0x20, 0x18, 0x20, 0x7F,}, // W
{ 0x63, 0x14, 0x08, 0x14, 0x63,}, // X
{ 0x03, 0x04, 0x78, 0x04, 0x03,}, // Y
{ 0x61, 0x51, 0x49, 0x45, 0x43,}, // Z
{ 0x7D, 0x12, 0x12, 0x7D, 0x00,}, // Ä
{ 0x3D, 0x42, 0x42, 0x42, 0x3D,}, // Ö
{ 0x3D, 0x40, 0x40, 0x40, 0x3D,}, // Ü
{ 0x00, 0x00, 0x7F, 0x41, 0x41,}, // [
{ 0x02, 0x04, 0x08, 0x10, 0x20,}, // "\"
{ 0x41, 0x41, 0x7F, 0x00, 0x00,}, // ]
{ 0x04, 0x02, 0x01, 0x02, 0x04,}, // ^
{ 0x40, 0x40, 0x40, 0x40, 0x40,}, // _
{ 0x00, 0x01, 0x02, 0x04, 0x00,}, // `
{ 0x20, 0x54, 0x54, 0x54, 0x78,}, // a
{ 0x7F, 0x48, 0x44, 0x44, 0x38,}, // b
{ 0x38, 0x44, 0x44, 0x44, 0x20,}, // c
{ 0x38, 0x44, 0x44, 0x48, 0x7F,}, // d
{ 0x38, 0x54, 0x54, 0x54, 0x18,}, // e
{ 0x08, 0x7E, 0x09, 0x01, 0x02,}, // f
{ 0x08, 0x14, 0x54, 0x54, 0x3C,}, // g
{ 0x7F, 0x08, 0x04, 0x04, 0x78,}, // h
{ 0x00, 0x44, 0x7D, 0x40, 0x00,}, // i
{ 0x20, 0x40, 0x44, 0x3D, 0x00,}, // j
{ 0x00, 0x7F, 0x10, 0x28, 0x44,}, // k
{ 0x00, 0x41, 0x7F, 0x40, 0x00,}, // l
{ 0x7C, 0x04, 0x18, 0x04, 0x78,}, // m
{ 0x7C, 0x08, 0x04, 0x04, 0x78,}, // n
{ 0x38, 0x44, 0x44, 0x44, 0x38,}, // o
{ 0x7C, 0x14, 0x14, 0x14, 0x08,}, // p
{ 0x08, 0x14, 0x14, 0x18, 0x7C,}, // q
{ 0x7C, 0x08, 0x04, 0x04, 0x08,}, // r
{ 0x48, 0x54, 0x54, 0x54, 0x20,}, // s
{ 0x04, 0x3F, 0x44, 0x40, 0x20,}, // t
{ 0x3C, 0x40, 0x40, 0x20, 0x7C,}, // u
{ 0x1C, 0x20, 0x40, 0x20, 0x1C,}, // v
{ 0x3C, 0x40, 0x30, 0x40, 0x3C,}, // w
{ 0x44, 0x28, 0x10, 0x28, 0x44,}, // x
{ 0x0C, 0x50, 0x50, 0x50, 0x3C,}, // y
{ 0x44, 0x64, 0x54, 0x4C, 0x44,}, // z
{ 0x20, 0x55, 0x54, 0x55, 0x78,}, // ä
{ 0x3A, 0x44, 0x44, 0x3A, 0x00,}, // ö
{ 0x3A, 0x40, 0x40, 0x3A, 0x00,}, // ü
{ 0x00, 0x08, 0x36, 0x41, 0x00,}, // {
{ 0x00, 0x00, 0x7F, 0x00, 0x00,}, // |
{ 0x00, 0x41, 0x36, 0x08, 0x00,}, // }
{ 0x14, 0x3E, 0x55, 0x41, 0x22,}, // €
{ 0x08, 0x08, 0x2A, 0x1C, 0x08,}, // -> (ALT + 0134) † 
{ 0x08, 0x1C, 0x2A, 0x08, 0x08,}, // <- (ALT + 0135) ‡
{ 0x00, 0x00, 0x07, 0x05, 0x07 } // °


    // Add more characters as needed
};
const uint8_t font[27][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x00, 0x00, 0x00, 0x00}  // Space
};

void oled_draw_char(uint8_t x, uint8_t y, char c) {
    if (c < 'A' || c > 'Z') c = 'Z' + 1;  // If not an uppercase letter, use space
    const uint8_t *bitmap = font[c - 'A'];

    for (uint8_t i = 0; i < 5; i++) {
        oled_command(0x15);  // Set column address
        oled_command(x + i);
        oled_command(x + i);
        oled_command(0x75);  // Set row address
        oled_command(y);
        oled_command(y + 7);

        oled_data(bitmap[i]);
    }
}

void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        oled_draw_char(x, y, *str++);
        x += 6;  // Move to the next character position (5 pixels per character + 1 pixel space)
        if (x >= 128) break;  // Stop if we reach the end of the screen
    }
}

void oled_char(char c) {
    if (c < 32 || c > 126) return;  // Unsupported character
    for (uint8_t i = 0; i < 5; i++) {
        oled_data(pgm_read_byte(&font5x7[c - 32][i]));
    }
    oled_data(0x00);  // Space between characters
}


void oled_string(const char *str) {
    while (*str) {
        oled_char(*str++);
    }
}
void oled_set_cursor(uint8_t column, uint8_t row) {
    oled_command(0x15);  // Set column address
    oled_command(column);
    oled_command(0x7F);
    oled_command(0x75);  // Set row address
    oled_command(row);
    oled_command(0x7F);
}

void display_test_pattern(void) {
    for (uint8_t page = 0; page < 8; page++) {
        oled_command(0xB0 + page);  // Set page address
        oled_command(0x00);  // Set lower column address
        oled_command(0x10);  // Set higher column address
        for (uint8_t col = 0; col < 128; col++) {
            oled_data(0xFF);  // Example data
        }
    }
}

void oled_init(void) {
    // Example initialization sequence (refer to SSD1327 datasheet)
    _delay_ms(300);
    /*
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
    */
      /*
    oled_command(0xAE);  // Display off
    oled_command(0x15);  // Set column address
    oled_command(0x00);
    oled_command(0x7F);
    oled_command(0x75);  // Set row address
    oled_command(0x00);
    oled_command(0x7F);
    // Additional initialization commands go here
    oled_command(0xAF);  // Display on
    */

    /*
    oled_command(0xAE);  // Display off
    oled_command(0x81);  // Set contrast control
    oled_command(0x80);  // Contrast value
    oled_command(0xA0);  // Segment remap
    oled_command(0x51);  // Enable external VSL
    oled_command(0xA1);  // Set display start line
    oled_command(0x00);
    oled_command(0xA2);  // Set display offset
    oled_command(0x00);
    oled_command(0xA4);  // Normal display
    oled_command(0xA8);  // Set multiplex ratio
    oled_command(0x3F);  // Duty = 1/64
    oled_command(0xAD);  // Set master configuration
    //oled_command(0x8E);  // External VCC supply // wird heller ohne
    oled_command(0xB0);  // Set page start address
    oled_command(0x00);
    oled_command(0xB1);  // Set phase length
    oled_command(0x31);
    oled_command(0xB3);  // Set display clock divide ratio/oscillator frequency
    oled_command(0xF0);  // FOSC
    oled_command(0x8A);  // Set pre-charge period
    oled_command(0x64);
    oled_command(0xB9);  // Use built-in linear LUT
    oled_command(0xBC);  // Pre-charge voltage
    oled_command(0x10);  // Pre-charge voltage level
    oled_command(0xBE);  // Set VCOMH
    oled_command(0x07);  // VCOMH value
    oled_command(0xAF);  // Display ON
    */
      /*
      // es gibt aktionen auf dem Screen, rauschen
    oled_command(0xAE);  // Display off
    oled_command(0x15);  // Set column address
    oled_command(0x00);
    oled_command(0x7F);
    oled_command(0x75);  // Set row address
    oled_command(0x00);
    oled_command(0x7F);
    oled_command(0x81);  // Set contrast control
    oled_command(0x80);
    oled_command(0xA0);  // Segment remap
    oled_command(0x51);
    oled_command(0xA1);  // Set display start line
    oled_command(0x00);
    oled_command(0xA2);  // Set display offset
    oled_command(0x00);
    oled_command(0xA4);  // Normal display
    oled_command(0xA8);  // Set multiplex ratio
    oled_command(0x3F);
    oled_command(0xB1);  // Set phase length
    oled_command(0xF1);
    oled_command(0xB3);  // Set display clock divide ratio
    oled_command(0x00);
    oled_command(0xAB);  // Enable internal VDD regulator
    oled_command(0x01);
    oled_command(0xB6);  // Set second pre-charge period
    oled_command(0x08);
    oled_command(0xBE);  // Set VCOMH
    oled_command(0x07);
    oled_command(0xBC);  // Set pre-charge voltage
    oled_command(0x1F);
    oled_command(0xAF);  // Display on
*/
/*
// full screen, schraeges Muster
oled_command(0xAE);  // Display off
    oled_command(0x15);  // Set column address
    oled_command(0x00);  // Start column
    oled_command(0x7F);  // End column
    oled_command(0x75);  // Set row address
    oled_command(0x00);  // Start row
    oled_command(0x7F);  // End row
    oled_command(0x81);  // Set contrast control
    oled_command(0x80);  // Contrast value
    oled_command(0xA0);  // Set re-map and data format
    oled_command(0x74);  // Horizontal address increment
    oled_command(0xA1);  // Set display start line
    oled_command(0x00);  // Start line
    oled_command(0xA2);  // Set display offset
    oled_command(0x00);  // Offset value
    oled_command(0xA4);  // Normal display
    oled_command(0xA8);  // Set multiplex ratio
    oled_command(0x7F);  // 1/128 duty
    oled_command(0xB3);  // Set display clock divide ratio/oscillator frequency
    oled_command(0xF0);  // Divide ratio
    oled_command(0x8A);  // Set Master Configuration
    oled_command(0x64);  // Configuration value
    oled_command(0xD5);  // Set Function Selection
    oled_command(0xA1);  // Enable internal VDD regulator
    oled_command(0xD9);  // Set pre-charge period
    oled_command(0x22);  // Pre-charge period
    oled_command(0xDB);  // Set VCOMH deselect level
    oled_command(0x30);  // VCOMH level
    oled_command(0xAD);  // Set second pre-charge period
    oled_command(0x8A);  // Second pre-charge period
    oled_command(0xAF);  // Display on
    */
    
    // 
   oled_command(0xAE);
   oled_command(0x81);
   oled_command(0x80);
   oled_command(0xA0);
   oled_command(0x51);
   oled_command(0xA1);
   oled_command(0x00);
   oled_command(0xA2);
   oled_command(0x00);
   oled_command(0xA6);
   oled_command(0x7F);
   oled_command(0xB1);
   oled_command(0x11);
   oled_command(0xB3);
   oled_command(0x00);
   oled_command(0xAB);
   oled_command(0x01);
   oled_command(0xB6);
   oled_command(0x04);
   oled_command(0x0F);
   oled_command(0xBC);
   oled_command(0x08);
   oled_command(0xD5);
   oled_command(0x62);
   oled_command(0xFD);
   oled_command(0x12);
   oled_command(0xA4);
   oled_command(0xAF);
   
   /*
   // waveshare OLED_Show
   oled_command(0xae);//--turn off oled panel
   oled_command(0x15);    //   set column address
   oled_command(0x00);    //  start column   0
   oled_command(0x7f);    //  end column   127
   oled_command(0x75);    //   set row address
   oled_command(0x00);    //  start row   0
   oled_command(0x7f);    //  end row   127
   oled_command(0x81);  // set contrast control
   oled_command(0x80);
   oled_command(0xa0);    // gment remap
   oled_command(0x51);   //51
   oled_command(0xa1);  // start line
   oled_command(0x00);
   oled_command(0xa2);  // display offset
   oled_command(0x00);
   oled_command(0xa4);    // rmal display
   oled_command(0xa8);    // set multiplex ratio
   oled_command(0x7f);
   oled_command(0xb1);  // set phase leghth
   oled_command(0xf1);
   oled_command(0xb3);  // set dclk
   oled_command(0x00);  //80Hz:0xc1 90Hz:0xe1   100Hz:0x00   110Hz:0x30 120Hz:0x50   130Hz:0x70     01
   oled_command(0xab);  //
   oled_command(0x01);  //
   oled_command(0xb6);  // set phase leghth
   oled_command(0x0f);
   oled_command(0xbe);
   oled_command(0x0f);
   oled_command(0xbc);
   oled_command(0x08);
   oled_command(0xd5);
   oled_command(0x62);
   oled_command(0xfd);
   oled_command(0x12);
   oled_command(0xAF);
*/
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
   uint8_t pos = 0;
	slaveinit();
  	
	// initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	//lcd_puts("Guten Tag\0");
	_delay_ms(1000);
	//lcd_cls();
	lcd_puts("A328_PIO_Start");
   //_delay_ms(1000);
	_delay_ms(200);
   //_delay_ms(1000);
   lcd_putc('A'+pos++);
   i2c_init();
   _delay_ms(200);
   _delay_ms(2);
   lcd_putc('A'+pos++);
   oled_init();
   _delay_ms(200);
   lcd_putc('A'+pos++);
   //_delay_ms(1000);
	
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   uint16_t loopcount2=0;

	
   sei();
   // Adafruit
   //display.begin(0x3D, OLED_RESET);
   //display.clearDisplay();
   //display.display();
   //display.drawPixel(10, 10, SSD1327_WHITE);
   lcd_putc('A'+pos++);
   pos = 0x0;
   uint8_t page = 0;
   //oled_command(0x21); // Column address
   //oled_command(0x00); // Column start address
	while (1)
   {  
      //OSZI_B_LO();
         loopcount0++;
         if (loopcount0>=2*refreshtakt)
         {
            loopcount0=0;
            loopcount1++;
            if (loopcount1>=refreshtakt)
            {
               loopcount1 = 0;
               LOOPLEDPORT ^= (1<<LOOPLED); 
               lcd_gotoxy(0,1);
               lcd_putint(loopcount2);
               lcd_putc(' ');
               lcd_putint(pos & 0xF);
                lcd_putc(' ');
               lcd_putint(page);
               loopcount2++;

               // OLED
               SYNC_LO();
               
               
               SYNC_HI();

               if (loopcount2 > 2)
               {
                  loopcount2 = 0;
                  /*
                  oled_command(0x21); // Column address
                  oled_command(0x00); // Column start address
                  oled_command(0x7F); // Column end address
                  oled_command(0x22); // Page address
                  oled_command(0x00); // Page start address
                  oled_command(0x07); // Page end address
                  */

                  //oled_command(0x22); // Page address
                  //oled_command(0x00 + page++); // Page start address

                  //for (uint16_t i = 0; i < 8; i++) 
                  {
                     oled_set_cursor(0,page);
                     oled_char('A');
                     oled_set_cursor(10,page);
                     oled_char('B'); // 
                     oled_set_cursor(20,page);
                     oled_char('C');
                     oled_set_cursor(30,page);
                     oled_char('D');
                  }
                  pos++;
                  pos &= 0x0F;
                  page+= 2;


               }
               

            }
            
            
            loopcount0=0;
            

            
            
            
            
            
            //OSZI_B_HI();
         }  // loopcount0>=refreshtakt
         OSZI_B_HI();
         
      
   }//while


 return 0;
}

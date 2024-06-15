/*----------------------------------------------------------------------------
 Copyright:
 Author:         Radig Ulrich
 Remarks:        
 known Problems: none
 Version:        21.11.2009
 Description:    Description:    EA DOG (M/L)128-6
------------------------------------------------------------------------------*/
// von RC_LCD_Teensy_13 (RC_PWM)


#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include "display.h"
#include "font.h"
#include "text.h"
#include "defines.h"

#include <String.h>

extern  volatile uint8_t char_x;
extern	volatile uint8_t char_y;
extern	volatile uint8_t char_height_mul;
//extern	volatile uint8_t char_width_mul;

extern uint8_t LOK_ADRESSE;

extern volatile uint8_t levelwert;
extern volatile uint8_t levelb;


extern volatile uint8_t expowert;
extern volatile uint8_t expob;


extern volatile uint16_t      laufsekunde;

//extern volatile uint8_t       curr_settingarray[8][2];
extern volatile uint8_t       curr_levelarray[8];
extern volatile uint8_t       curr_expoarray[8];
extern volatile uint8_t       curr_mixarray[8];
extern volatile uint8_t       curr_funktionarray[8];
extern volatile uint8_t       curr_devicearray[8];
extern volatile uint8_t       curr_ausgangarray[8];



extern volatile uint8_t       curr_screen;
extern volatile uint8_t       curr_page; // aktuelle page
extern volatile uint8_t       curr_col; // aktuelle colonne
extern volatile uint8_t       curr_model; // aktuelles modell

extern volatile uint8_t       curr_kanal; // aktueller kanal

extern volatile uint8_t       curr_richtung; // aktuelle richtung
extern volatile uint8_t       curr_impuls; // aktueller impuls

extern volatile uint8_t       eepromsavestatus;
extern volatile uint8_t       programmstatus;

extern volatile uint8_t       curr_setting;
extern volatile uint8_t       curr_cursorzeile; // aktuelle zeile des cursors
extern volatile uint8_t       curr_cursorspalte; // aktuelle colonne des cursors

extern volatile uint8_t       last_cursorzeile; // letzte zeile des cursors
extern volatile uint8_t       last_cursorspalte; // letzte colonne des cursors
extern volatile uint16_t      blink_cursorpos;


extern volatile uint8_t       itemtab[10];
extern volatile uint8_t      cursortab[10];
extern volatile uint16_t      manuelltrimmcounter;

extern volatile uint8_t       blinkcounter;

extern volatile uint8_t       laufstunde;
extern volatile uint8_t       laufminute;
extern volatile uint16_t      motorsekunde;
extern volatile uint16_t      stopsekunde;
extern volatile uint16_t      motorminute;
extern volatile uint16_t      stopminute;
extern volatile uint16_t      batteriespannung;

extern volatile uint16_t      tastentransfer;

extern volatile uint16_t      posregister[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

extern volatile uint16_t      cursorpos[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer cursor (hex). geladen aus progmem

extern uint16_t spicounter;

// 
extern volatile uint16_t      updatecounter; // Zaehler fuer Einschalten
/*
 write_dogm(0x40,A0);// Display start line set --> 0
 write_dogm(0xA1,A0);  // ADC set --> reverse
 write_dogm(0xC0,A0);  // Common output mode select --> normal
 write_dogm(0xA6,A0);  // Display --> normal
 write_dogm(0xA2,A0);  // LCD Bias set --> 1/9 (Duty 1/65)
 write_dogm(0x2F,A0);  // Power control set --> Booster, Regulator and Follower on
 write_dogm(0xF8,A0);  // Booster ratio set --> Set internal Booster to 4x
 write_dogm(0x00,A0);  // ...
 write_dogm(0x27,A0);  // Contrast set
 write_dogm(0x81,A0);  // ...
 write_dogm(actContrast,A0);
 write_dogm(0xAC,A0);  // Static indicator set --> no indicator
 write_dogm(0x00,A0);  // ...
 write_dogm(0xAF,A0);  // Display on/off

 */
#define HOMESCREEN      0
#define SETTINGSCREEN   1
#define KANALSCREEN     2
#define LEVELSCREEN     3
#define EXPOSCREEN      4
#define MIXSCREEN       5
#define ZUTEILUNGSCREEN 6
#define AUSGANGSCREEN   7
#define SAVESCREEN      8
#define TRIMMSCREEN      9

#define MODELLCURSOR 2
#define SETCURSOR    4
#define KANALCURSOR  6
#define MIXCURSOR    7

#define KANALTEXTCURSOR 1
#define KANALNUMMERCURSOR  3

#define TRIMMTIMEOUT	10 // Loopled-counts bis Manuell zurueckgesetzt wird. 50: ca. 30s

// Homescreen

const PROGMEM uint8_t titel0[]  = "H0_Decoder328\0";
const PROGMEM uint8_t titel1[]  = "ON-Zeit:\0";
const PROGMEM uint8_t titel2[]  = "Stoppuhr\0";
const PROGMEM uint8_t titel3[]  = "Motorzeit\0";
const PROGMEM uint8_t titel4[]  = "Menu";
const PROGMEM uint8_t titel5[]  = "Set";
const PROGMEM uint8_t titel6[]  = "Akku\0";
const PROGMEM uint8_t titel7[]  = "D\0";

const  uint8_t*   TitelTable[]  = {titel0, titel1, titel2, titel3, titel4, titel5, titel6, titel7};

volatile uint8_t itemtab[10] = {8+RANDLINKS,32+RANDLINKS,48+RANDLINKS,60+RANDLINKS,72+RANDLINKS,84+RANDLINKS,96+RANDLINKS,108+RANDLINKS,116+RANDLINKS,0+RANDLINKS};

volatile uint8_t cursortab[10] = {0+RANDLINKS,24+RANDLINKS+RANDLINKS,40+RANDLINKS,52+RANDLINKS,64+RANDLINKS,76+RANDLINKS,88+RANDLINKS,100+RANDLINKS,108+RANDLINKS,0};



uint8_t balken[8]=
{0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF};

const uint8_t marke[8]=
{0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};

const uint8_t marke_dick[8]=
{0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x03};



// EA DOGL128 // Text
/*
const volatile char DISPLAY_INIT[] =
{
   0x40,// Display start line set --> 0
   
//   0xA1, // ADC set --> reverse Horizontal gespiegelt wenn A1
   0xA0, // ADC set --> reverse Horizontal gespiegelt wenn A1
   
//   0xC0, // Common output mode select --> normal
   0xC8, // Common output mode select --> normal
   
   
   0xA6, // Display --> normal  A7 ist reverse
   0xA2, // LCD Bias set --> 1/9 (Duty 1/65)
   0x2F, // Power control set --> Booster, Regulator and Follower on
   0xF8, // Booster ratio set --> Set internal Booster to 4x
   0x00, // ...
   0x27, //
   0x81, // Contrast set
   0x10,
   0xAC, // Static indicator set --> no indicator
   0x00, // ...
   0xAF
};  // Display on/off
 */
// Orientierung 6:00 Uhr
const uint8_t DISPLAY_INIT[] = {0x40, 0xA0, 0xC8, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x10, 0xAC, 0x00, 0xAF};

// Orientierung 12:00 Uhr: offset 4 zu x addieren: RANDLINKS
//const char DISPLAY_INIT[] = {0x40, 0xA1, 0xC0, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x10, 0xAC, 0x00, 0xAF};


// von DOGM204_SSD1803A_SPI

//const char DISPLAY_INIT[] = {0x3A,0x09,0x06,0x1E,0x39,0x1B,0x6E,0x57,0x72,0x38,0x0F};


//const char DISPLAY_INIT[] = {0xC8,0xA1,0xA6,0xA2,0x2F,0x26,0x81,0x25,0xAC,0x00,0xAF};

//char_height_mul 1, 2 or 4
//volatile uint8_t char_x=0,char_y=1,char_height_mul=1,char_width_mul=1;

void resetRegister(void)
{
   uint8_t i=0,k=0;
   for(i=0;i<8;i++)
   {
      for (k=0;k<8;k++)
      {
         posregister[i][k]=0xFFFF;
      }
      
   }
}

//=============================================================================
//Basic Byte Access to Display
//=============================================================================
/*
//Writes one byte to the data ram
void lcd_data(uint8_t data) {
  LCD_SELECT();
  LCD_DRAM();
  spi_write(data);
  LCD_UNSELECT();
  lcd_inc_column(1);
  }

//Writes one command byte
void lcd_command(uint8_t cmd) {
  LCD_SELECT();
  LCD_CMD();
  spi_write(cmd);
  LCD_UNSELECT();
  }
  */

void setlogscreen(void)
{
   display_clear();
   resetRegister();
   posregister[0][0] = itemtab[5] | (1 << 8);// Laufzeit Anzeige
   char_x=RANDLINKS;
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_go_to(char_x,char_y);
   uint16_t dat = pgm_read_word_near(&(TitelTable[0]));
   strcpy((char*)titelbuffer,(const char*)dat);
   char_height_mul = 1;
   //char_width_mul = 2;
   display_write_str((const unsigned char*)titelbuffer,1);

   char_x=RANDLINKS;
   char_y = 2;
   display_write_str((const unsigned char*)"Lok-Adresse",1);
   char_height_mul = 1;
   char_width_mul = 1;
   //display_go_to(0,4);
   char_x = 64;
   char_y = 2;
   display_write_int(LOK_ADRESSE,1);
   
   display_write_str((const unsigned char*)" ",1);
   display_write_hex(LOK_ADRESSE,1);
   //display_go_to(10,4);
   //display_write_propchar('A',1);

  }

void sethomescreen(void)
{
   // Laufzeit
   resetRegister();
   posregister[0][0] = itemtab[5] | (1 << 8);// Laufzeit Anzeige
   
   
   posregister[1][0] = (0+RANDLINKS) | (0x05 << 8); // Text Motorzeit
   posregister[1][1] = (0+RANDLINKS) | (0x06 << 8); // Anzeige Motorzeit
   
   
   posregister[2][0] = (60+RANDLINKS) | (0x05 << 8); // Text Stoppuhr
   posregister[2][1] = (60+RANDLINKS) | (0x06 << 8); // Anzeige Stoppuhr
   
   
   posregister[3][0] = (60+RANDLINKS) | (0x07 << 8); // Text Akku
   posregister[3][1] = (84+RANDLINKS) | (0x08 << 8); // Anzeige Akku

   posregister[4][0] = (0+RANDLINKS) | (2 << 8); // Name Modell
   posregister[4][1] = (80+RANDLINKS) | (3 << 8); // Text Setting
   posregister[4][2] = (100+RANDLINKS) | (3 << 8); // Anzeige Setting

   
  
   // positionen lesen
   // titel setzen
   uint16_t dat = pgm_read_word_near(&(TitelTable[0]));
   strcpy_P((char*)titelbuffer, (const char*) dat);
   char_x=RANDLINKS;
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_go_to(char_x+1,0);
   display_write_byte(DATA,0xFF);
   //char_x++;

   display_inverse(1);
   //display_write_prop_str(char_y,char_x,0,(uint8_t*)titelbuffer);
   display_write_inv_str(titelbuffer,1);
   display_inverse(0);
   char_height_mul = 1;
   char_width_mul = 1;
   
   // Stoppuhrtext schreiben
   dat = pgm_read_word_near(&(TitelTable[2]));
   strcpy_P((char*)titelbuffer, (const char*)dat); // Text Stoppuhr
   char_x = (posregister[2][0] & 0x00FF);
   char_y= (posregister[2][0] & 0xFF00)>>8;
   display_write_str((const unsigned char*)titelbuffer,2);
   
   // Stoppzeit schreiben
   char_y= (posregister[2][1] & 0xFF00)>>8;
   char_x = (posregister[2][1] & 0x00FF);
   char_height_mul = 2;
   char_width_mul = 2;
   display_write_stopzeit_BM(stopsekunde,stopminute);
   char_height_mul = 1;
   char_width_mul = 1;

   
   // Motorzeittext schreiben
   dat = pgm_read_word_near(&(TitelTable[3]));
   strcpy((char*)titelbuffer,(const char*)dat);
   char_x = (posregister[1][0] & 0x00FF);
   char_y= ((posregister[1][0] & 0xFF00)>>8);
   char_height_mul = 1;
   display_write_str((const unsigned char*)titelbuffer,2);
   char_height_mul = 2;
   char_width_mul = 2;

   char_y= (posregister[1][1] & 0xFF00)>>8;
   char_x = posregister[1][1] & 0x00FF;
   // display_write_min_sek(motorsekunde,2);
   display_write_stopzeit(motorsekunde,motorminute, 2);
   
   char_height_mul = 1;
   char_width_mul = 1;

   
  
   // Batteriespannung
   char_y= ((posregister[3][0] & 0xFF00)>>8)+1;
   char_x = posregister[3][0] & 0x00FF;
   dat = pgm_read_word_near(&(TitelTable[6]));
   strcpy((char*)titelbuffer,(const char*)dat);
   char_height_mul = 1;
   display_write_str((const unsigned char*)titelbuffer,2);
   
   char_height_mul = 1;
   char_width_mul = 1;

   
}// sethomescreen





void update_time(void)
{
   char_x = posregister[0][0] & 0x00FF;
   char_y= (posregister[0][0] & 0xFF00)>>8;
   
   char_x = 24;
   char_y = 4;
   
   char_height_mul = 1;
   char_width_mul = 1;
   
   //display_write_min_sek(laufsekunde, 2);
   display_write_zeit(laufsekunde&0xFF,laufminute,laufstunde, 2);
   
   // Stoppzeit aktualisieren
   char_y= (posregister[2][1] & 0xFF00)>>8;
   char_x = posregister[2][1] & 0x00FF;
   char_height_mul = 2;
   char_width_mul = 2;
   //display_write_min_sek(stopsekunde,2);
   display_write_stopzeit(stopsekunde,stopminute, 2);
   
   // Motorzeit aktualisieren
   char_height_mul = 2;
   char_width_mul = 2;
   char_y= (posregister[1][1] & 0xFF00)>>8;
   char_x = posregister[1][1] & 0x00FF;
   //display_write_min_sek(motorsekunde,2);
   display_write_stopzeit(motorsekunde,motorminute, 2);

   // Batteriespannung aktualisieren
   char_y= (posregister[3][1] & 0xFF00)>>8;
   char_x = posregister[3][1] & 0x00FF;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_spannung(batteriespannung/10,2);
   // Akkubalken anzeigen
   char_height_mul = 1;
   char_width_mul = 1;
   display_akkuanzeige(batteriespannung);


}

void update_akku(void)
{
   
}


uint8_t update_screen(void)
{
   uint8_t fehler=0;
   //uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   fehler=1;
   switch (curr_screen)
   {
         
      case HOMESCREEN: // homescreen
      {
// MARK:  update HOMESCREEN
         
         fehler=2;
         updatecounter++;
         //Laufzeit
         char_x = posregister[0][0] & 0x00FF;
         char_y= (posregister[0][0] & 0xFF00)>>8;
         char_height_mul = 1;
         char_width_mul = 1;
         display_write_zeit(laufsekunde&0xFF,laufminute,laufstunde, 2);
         
         //display_write_min_sek(laufsekunde, 2);
         // Stoppzeit aktualisieren
         char_y= (posregister[2][1] & 0xFF00)>>8;
         char_x = posregister[2][1] & 0x00FF;
         char_height_mul = 2;
         char_width_mul = 2;
         //        if (programmstatus &(1<<STOP_ON)) // loescht nicht bei reset
         {
            display_write_stopzeit_BM(stopsekunde,stopminute);
         }
         
         // Motorzeit aktualisieren
         //         char_height_mul = 2;
         //         char_width_mul = 2;
         char_y= (posregister[1][1] & 0xFF00)>>8;
         char_x = posregister[1][1] & 0x00FF;
         {
            display_write_stopzeit_BM(motorsekunde,motorminute);
         }
         
         // Batteriespannung aktualisieren
         char_y= (posregister[3][1] & 0xFF00)>>8;
         char_x = posregister[3][1] & 0x00FF;
         char_height_mul = 1;
         char_width_mul = 1;
         display_write_spannung(batteriespannung/10,2);
         
                  
         // Akkubalken anzeigen
         //        char_height_mul = 1;
         //        char_width_mul = 1;
         //         display_akkuanzeige(batteriespannung);
         
         //    display_trimmanzeige_horizontal (4+RANDLINKS,3, 4,-15);
         //if (trimmstatus == 2)
         
      }
          
          
          
          
           
         
 


// MARK:  update TRIMMSCREEN

 

   }
   return fehler;
}

//##############################################################################################
// Curser weg an curr position
//
//##############################################################################################
void display_cursorweg(void)
{
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   char_y= (cursorposition & 0xFF00)>>8;
   char_x = cursorposition & 0x00FF;
   display_write_symbol(pfeilwegrechts);
}

//##############################################################################################
// Trimmanzeige
//
//##############################################################################################
void display_trimmanzeige_horizontal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition)
{
   //mitteposition ist Abweichung von Mitte des Kanals, mit Vorzeichen
   
   uint8_t col=char_x0, page=char_y0, breite=60;
   
   // linke Begrenzung
   display_go_to(col,page);
   
   uint8_t i=0;
   for (i=0;i<breite;i++)
   {
      if ((i==0)||(i==breite-1) )// || (i== breite/2+mitteposition))//|| (i== breite/2+mitteposition-1)|| (i== breite/2+mitteposition+1))
      {
         //display_write_byte(DATA,0x7E);// Begrenzungsstrich zeichnen
      }
      else if (i==breite/2)
      {
         display_write_byte(DATA,0xFF);// Mittelmarke zeichnen
      }
      else if ((i== breite/2+mitteposition)|| (i== breite/2+mitteposition)|| (i== breite/2+mitteposition-1))
      {
         display_write_byte(DATA,0x7E);
      }
      
      
      else
      {
         //display_write_byte(DATA,0x42);// obere und untere linie  zeichnen
         display_write_byte(DATA,0x18);
      }
   }

}

void display_trimmanzeige_horizontal_weg (uint8_t char_x0, uint8_t char_y0)
{
   //Anzeige entfernen
   
   uint8_t col=char_x0, page=char_y0, breite=60;
   
   // linke Begrenzung
   display_go_to(col,page);
   
   uint8_t i=0;
   for (i=0;i<breite;i++)
   {
         display_write_byte(DATA,0x00);// leer
      
    }
   
}





void display_trimmanzeige_vertikal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition)
{
   //mitteposition ist Abweichung von Mitte des Kanals, mit Vorzeichen
   
   uint8_t col=char_x0, page=char_y0, breite=6; uint8_t hoehe = 6;
   
   int16_t tempmitteposition = mitteposition;
   // auf hoehe normieren
   tempmitteposition = tempmitteposition*(hoehe*4)/0x7F; // hoehe*8: Hoehe Balken in pix. /2: Mitte in pix
   uint16_t markenlage =(hoehe*4 + tempmitteposition);
   
   //display_go_to(col,page);
   int8_t full =markenlage/8; // ganze pages
   int8_t part =markenlage%8; // rest
   //uint8_t balkenbreite = 6;
   
   //full=2;
   //part=4;
   
   // linke Begrenzung
   /*
   display_go_to(0,3);
   display_write_int(mitteposition,2);
   display_write_propchar(' ',2);
   
   display_write_int(full,2);
   display_write_propchar(' ',2);
   display_write_int(part,2);
   
   return;
    */
   //display_go_to(col,page);
   for (page=char_y0;page > char_y0-(hoehe);page--)
   {
      if (page == char_y0-hoehe/2)// Mitte
      {
         display_go_to(char_x0-1,page);
         display_write_byte(DATA,0x80);
         display_go_to(char_x0+breite,page);
         display_write_byte(DATA,0x80);
      }

      for (col=char_x0;col<char_x0+breite;col++)
      {
         display_go_to(col,page);
         
         //if ((col==char_x0)||(col==char_x0+breite-1) )
         if ((col==char_x0+breite/2)||(col==char_x0+breite/2-1)  ) // Mittelstrich zeichnen
         {
            display_write_byte(DATA,0xFF);// senkrechte Begrenzung zeichnen
            
         }
         
         else
         {
            
            uint8_t markenwert=0;
            
            if (page == (char_y0-full)) // Wert liegt in der page
            //if (page == (2)) // Wert liegt in der page
            {
               markenwert = marke[part];
            }
            if (page == char_y0) // untere Begrenzung
            {
               //markenwert |= 0x80;
            }
           // else if (page == 1)  // obere Begrenzung

            else if (page == char_y0+1-hoehe)  // obere Begrenzung
            {
               //markenwert |= 0x01;
            }
            
            else if (page == char_y0-hoehe/2)// Mitte
            {
               markenwert |= 0x80;
            }
            
            
            display_write_byte(DATA,markenwert);
         }
         
      }
      
   }
  // display_go_to(char_x0,0);
   
   
}


void display_trimmanzeige_vertikal_weg (uint8_t char_x0, uint8_t char_y0)
{
   //mitteposition ist Abweichung von Mitte des Kanals, mit Vorzeichen
   
   uint8_t col=char_x0, page=char_y0, breite=6; uint8_t hoehe = 6;
   for (page=char_y0;page > char_y0-(hoehe);page--)
   {
      
      for (col=char_x0-1;col<char_x0+breite+1;col++)
      {
         display_go_to(col,page);
         display_write_byte(DATA,0x00);
      }
   }
}

//##############################################################################################
// Akkuanzeige
//
//##############################################################################################
void display_akkuanzeige (uint16_t spannung)
{
   uint16_t balkenhoehe =(spannung-MINSPANNUNG)*64/(MAXSPANNUNG-MINSPANNUNG);
   uint8_t col=0, page=0;
   uint8_t full =balkenhoehe/8; // page ist voll
   uint8_t part =balkenhoehe%8; // rest
   uint8_t balkenbreite = 12;
   uint8_t grenze = 4;
   //part=4;
   //full = 2;
   uint8_t char_x0 = 112+RANDLINKS;
   
   for (page=1;page<8;page++)
   {
      /*
      display_go_to(char_x+1,page);
      display_write_byte(DATA,0xAA);
      display_go_to(char_x+balkenbreite,page);
      display_write_byte(DATA,0xAA);
       */
      col=0;
      while(col++ < balkenbreite)
      {
         display_go_to(char_x0+col,page);
         
         if (page < (7-full)) // sicher
         {
            if (page == grenze) // Strich zeichnen
            {
               display_write_byte(DATA,0x80);
            }
            else // leer lassen
            {
               display_write_byte(DATA,00);
            }
         }
         else if (page == (7-full)) // grenzwertig
         {
            if ((full<grenze-1) && (laufsekunde%2)) // Blinken
            {
               display_write_byte(DATA,0x00);
            }
            else
            {
               if (page == grenze) // Strich zeichnen wenn unter Grenze, sonst luecke zeichnen
               {
                  //display_write_byte(DATA,(balken[part] | 0x08));
                  display_write_byte(DATA,(balken[part] ^ 0x80)); // war 0x80 fuer duenneren Strich
               }
               else if (page > grenze) // kein
               {
                  display_write_byte(DATA,(balken[part] ));
               }
               else
               {
                  display_write_byte(DATA,(balken[part] ));
               }
            }
            
         }
         else // wird unsicher
         {
            if (page == grenze) // grenzwertig
            {
               display_write_byte(DATA,0x7F); // Strich zeichnen
            }
            else
            {
               if ((full<grenze-1) && (laufsekunde%2)) // Blinken
               {
                  display_write_byte(DATA,0x00);
               }
               else
               {
                  display_write_byte(DATA,0xFF); // voller Balken
               }
            } // else if (page == grenze)
            
         } // else if(page == (7-full))
         
      } // while col
      

   } // for page
   
}

//##############################################################################################
//Diagramm
//
//##############################################################################################
uint8_t display_diagramm (uint8_t char_x, uint8_t char_y, uint8_t stufea, uint8_t stufeb, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;

   uint8_t maxX=50, maxY=48;
   //uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y;page>3;page--) //Ordinate
   {
      display_go_to(char_x-maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen

      display_go_to(char_x,page);
      display_write_byte(DATA,0xFF); // Strich zeichnen
      display_go_to(char_x+maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen

   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   for (col=1;col<maxX;col++)
   {
      wertYA = (4-stufea)*col*0x20/0x32/4;
      pageA = 7-(wertYA/8);
      wertYB = (4-stufeb)*col*0x20/0x32/4;
      pageB = 7-(wertYB/8);
      
      for (k=7; k >2; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(7-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(7-wertYB%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
         
         // Seite A (links)
      
         display_go_to(char_x-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
       

      
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATA,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return 1;
   
}


uint8_t display_kanaldiagramm (uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;
   
   uint8_t maxX=50, maxY=48;
   //uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y0;page>3;page--) //Ordinate
   {
      display_go_to(char_x0-maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen
      
      display_go_to(char_x0,page);
      display_write_byte(DATA,0xFF); // Strich zeichnen
      display_go_to(char_x0+maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen
      
   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   uint8_t expoa=((expo & 0x30)>>4);
   uint8_t expob=(expo & 0x03);
   for (col=1;col<maxX;col++)
   {
      if (expoa==0) // linear
      {
         wertYA = (8-((level & 0x70)>>4))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            // expoa wirkt erst ab wert 1, array der Werte ist 0-basiert: Wert an expoa-1 lesen
            wertYA = pgm_read_byte(&(expoarray25[expoa-1][col/2]))/2 +pgm_read_byte(&(expoarray25[expoa-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYA = pgm_read_byte(&(expoarray25[expoa-1][col/2]));
         }
         wertYA =(8-((level & 0x70)>>4))*wertYA/8; // Level
      }
      pageA = 7-(wertYA/8);
      
      
      
      if (expob==0) // linear
      {
         wertYB = (8-(level & 0x07))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            wertYB = pgm_read_byte(&(expoarray25[expob-1][col/2]))/2 +pgm_read_byte(&(expoarray25[expob-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYB = pgm_read_byte(&(expoarray25[expob-1][col/2]));
         }
         wertYB =(8-(level & 0x07))*wertYB/8; // Level
      }
      
      pageB = 7-(wertYB/8);
      
      for (k=7; k >2; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x0+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(7-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(7-wertYB%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATA,0x00); //Punkte entfernen
         }
         
         // Seite A (links)
         
         display_go_to(char_x0-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATA,0x00); //Punkte entfernen
         }
         
         
         
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATA,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return expob;
   
}

uint8_t display_kanaldiagramm_var (uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;
   
   uint8_t maxX=50, maxY=48;
   //uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y0;page>char_y0-4;page--) //Ordinate
   {
      display_go_to(char_x0-maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen
      
      display_go_to(char_x0,page);
      display_write_byte(DATA,0xFF); // Strich zeichnen
      display_go_to(char_x0+maxX,page);
      display_write_byte(DATA,0xDB); // Strich zeichnen
      
   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   uint8_t expoa=((expo & 0x30)>>4);
   uint8_t expob=(expo & 0x03);
   for (col=1;col<maxX;col++)
   {
      if (expoa==0) // linear
      {
      wertYA = (8-((level & 0x70)>>4))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            // expoa wirkt erst ab wert 1, array der Werte ist 0-basiert: Wert an expoa-1 lesen
            wertYA = pgm_read_byte(&(expoarray25[expoa-1][col/2]))/2 +pgm_read_byte(&(expoarray25[expoa-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYA = pgm_read_byte(&(expoarray25[expoa-1][col/2]));
         }
         wertYA =(8-((level & 0x70)>>4))*wertYA/8; // Level
      }
      pageA = char_y0-(wertYA/8);
      
      
      if (expob==0) // linear
      {
         wertYB = (8-(level & 0x07))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            wertYB = pgm_read_byte(&(expoarray25[expob-1][col/2]))/2 +pgm_read_byte(&(expoarray25[expob-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYB = pgm_read_byte(&(expoarray25[expob-1][col/2]));
         }
         wertYB =(8-(level & 0x07))*wertYB/8; // Level
      }
      
      pageB = char_y0-(wertYB/8);
      
      for (k=char_y0; k >char_y0-5; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x0+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(char_y0-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(char_y0-wertYB%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATA,0x00); //Punkte entfernen
         }
         
         // Seite A (links)
         
         display_go_to(char_x0-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATA,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATA,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATA,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATA,0x00); //Punkte entfernen
         }
      
         
         
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATA,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return expob;
   
}







//##############################################################################################
//Writes one byte to data or cmd register
//
//##############################################################################################
void display_back_char (void)
{
	char_x = char_x - (FONT_WIDTH*char_width_mul);
	if (char_x > 128+RANDLINKS) char_x = 0+RANDLINKS;
}

//##############################################################################################
//Writes one byte to data or cmd register
//
//##############################################################################################
uint8_t display_write_byte(unsigned cmd_data, uint8_t data) 
{
    // Hardware-SPI
   
   SPI_CS_LO();
   
	if(cmd_data == 0)
	{
      SPI_A0_HI();
      //DOG_SPI_PORT |= (1<<DOG_SPI_A0);
	}
	else
	{
      SPI_A0_LO();
      //DOG_SPI_PORT &= ~(1<<DOG_SPI_A0);
	}
   //OSZI_B_LO(); 
   spicounter = MAX_SPI_COUNT;
	SPDR = data;
	while((!(SPSR & (1<<SPIF))) && spicounter)
   {
      //OSZI_A_TOGG();
      spicounter--;
   }
   //OSZI_B_HI(); 
   _delay_us(1);
   SPI_CS_HI();
   
   return(spicounter > 0);
}

//##############################################################################################
void display_write_cmd(unsigned cmd_data)
{
   
   SPI_CS_LO();
   SPI_A0_LO(); // cmd
   SPDR = cmd_data;
   while((!(SPSR & (1<<SPIF))) && spicounter);
   _delay_us(1);
   
   SPI_CS_HI();
}

//##############################################################################################
void display_write_data(unsigned data)
{
   
   SPI_CS_LO();
   SPI_A0_HI(); // cmd
   SPDR = data;
   while((!(SPSR & (1<<SPIF))) && spicounter);
   _delay_us(1);
   
   SPI_CS_HI();
}




//##############################################################################################
//Init LC-Display
//
//##############################################################################################
uint8_t display_init() 
{
   /*
	//Set TIMER0 (PWM OC2 Pin)
	HG_PWM_DDR |= (1<<HG_PWM_PIN);//PWM PORT auf Ausgang (OC2)
	TCCR2A |= (1<<WGM21|1<<WGM20|1<<COM2A1|1<<CS20);
	OCR2A = 50;
*/
   SPI_RST_HI();
   SPI_CS_HI();
   _delay_ms(1);
   SPI_CS_LO();
   _delay_ms(1);
   SPI_RST_LO(); // reset
   _delay_ms(1);
   SPI_RST_HI();
   
	_delay_us(10);
	//send 11 init commands to Display
   uint8_t initerr = 0;
	for (uint8_t tmp = 0;tmp < 14;tmp++)
	{
      
      uint8_t a = display_write_byte(CMD,DISPLAY_INIT[tmp]);
      initerr += a;
      _delay_us(1);
	}
   
   SPI_CS_HI();
	display_clear();
	
	return initerr;
    
}

void display_soft_init(void)
{
	//Set TIMER0 (PWM OC2 Pin)
	TCCR2A |= (1<<WGM21|1<<WGM20|1<<COM2A1|1<<CS20);
	OCR2A = 50;
	
   SOFT_SPI_DDR |= (1<<DOG_A0);
   SOFT_SPI_PORT |= (1<<DOG_A0);
   
   SOFT_SPI_DDR |= (1<<DOG_RST);
   SOFT_SPI_PORT |= (1<<DOG_RST);
   
   SOFT_SPI_DDR |= (1<<DOG_CS);
   SOFT_SPI_PORT |= (1<<DOG_CS);
   
   SOFT_SPI_DDR |= (1<<DOG_SCL);
   SOFT_SPI_PORT &= ~(1<<DOG_SCL);
   
   SOFT_SPI_DDR |= (1<<DOG_DATA);
   SOFT_SPI_PORT &= ~(1<<DOG_DATA);

   SOFT_SPI_DDR |= (1<<DOG_PWM);
   SOFT_SPI_PORT &= ~(1<<DOG_PWM);


	_delay_us(1);
	//send 11 init commands to Display
   
	for (uint8_t tmp = 0;tmp < 14;tmp++)
	{
		display_write_byte(CMD,DISPLAY_INIT[tmp]);
      _delay_us(1);
	}
	display_clear();
	
	return;
}

//##############################################################################################
//Go to x,y
//
//##############################################################################################

void display_go_to (uint8_t x, uint8_t y)
{
   
	display_write_byte(CMD,DISPLAY_PAGE_ADDRESS | ((y) & 0x0F));
   
	display_write_byte(CMD,DISPLAY_COL_ADDRESS_MSB | ((x>>4) & 0x0F));
  
	display_write_byte(CMD,DISPLAY_COL_ADDRESS_LSB | ((x) & 0x0F));
	return;

}



//##############################################################################################
//Diplay clear
//
//##############################################################################################
void display_clear()
{
	uint8_t page, col;
	
	for(page=0;page<8;page++)
	{
		display_go_to(0+RANDLINKS,page);
	
		for (col=0+RANDLINKS;col<128+RANDLINKS;col++)
		{
         if (col%4)
         {
			display_write_byte(DATA,0x00);
         }
         else
         {
            display_write_byte(CMD,0x00);
         }
		}
	}
}


//##############################################################################################
//Diplay clear
//
//##############################################################################################
void display_inverse(uint8_t inv)
{
   display_write_byte(1,0xA6+(inv & 0x01));
}

//##############################################################################################
//Diplay Memory (BMP MONO 64x128)
//
//##############################################################################################
void display_mem(PGM_P pointer)
{
	uint8_t col,page;
	
	for(col=DISPLAY_OFFSET;col < 128 + DISPLAY_OFFSET;col++) 
	{
		for (page=8;page!=0;page--)
		{
			display_go_to(col,page-1);
				
			display_write_byte(0,~(pgm_read_byte(pointer++)));
		}
	}
}

//##############################################################################################
//Ausgabe eines Zeichens
//
//##############################################################################################
/*
void display_write_char(uint8_t c)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
				
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
				
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
				
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				} // end >2

			
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{	
				display_write_byte(DATA,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}
*/
// inv
/*
void  display_write_inv_char(uint8_t c)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,~tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATA,~tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}
*/

//##############################################################################################
//Ausgabe eines Prop Zeichens
//
//##############################################################################################
void display_write_propchar(uint8_t c, uint8_t prop)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
   
	const uint8_t* pointer;
   uint8_t charsize = 8;
   switch (prop)
   {
      case 1:
      {
         pointer = propfont6[c-32]; // 32 ist start in ascii
         //charsize = 6;
      }break;
         
      case 2:
      {
         pointer = propfont8[c-32];
      }break;
      default:
      {
          pointer = propfont6[c-32];
      }
   }
	uint8_t  charbreite =   pgm_read_byte(pointer++); // erste Zahl ist breite
   
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				} // end >2
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATA,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
      char_x++;
      display_write_byte(DATA,0x00);
      if (char_height_mul > 1)
      {
         display_go_to(char_x,char_y);
         display_write_byte(DATA,0x00);
      }
      //char_x++;
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}

// invertiert

void display_write_inv_propchar(uint8_t c, uint8_t prop)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
   
	const uint8_t* pointer;
   uint8_t charsize = 8;
   switch (prop)
   {
      case 1:
      {
         pointer = propfont6[c-32];
      }
      case 2:
      {
         pointer = propfont8[c-32];
         
      }break;
      default:
      {
         pointer = propfont6[c-32];
      }
   }
	uint8_t  charbreite =   pgm_read_byte(pointer++);
   
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp4);
					}
				} // end >2
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,~tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATA,~tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
      display_write_byte(DATA,0xFF);
      if (char_height_mul > 1)
      {
         display_go_to(col,page-1);
         display_write_byte(DATA,0xFF);
      }
      char_x++;
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}




//##############################################################################################
// Ausgabe simple char
/*
void display_write_simplechar(uint8_t c)
{
	uint8_t col,tmp1,page;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH)+DISPLAY_OFFSET);col=col+1)
	{
		{
			tmp1 = pgm_read_byte(pointer++);
			
			display_go_to(col,char_y);
         display_write_byte(DATA,tmp1);
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH);
	}
	return;
}
*/
//##############################################################################################
// Ausgabe simple prop char

void display_write_simple_propchar(uint8_t c, uint8_t prop, uint8_t offset)
{
	uint8_t col,page;
   uint16_t tmp1;
	const uint8_t* PROGMEM pointer = propfont8[c-32];
   
  
   //char_x = 60;
   uint8_t fontwidth =pgm_read_byte(pointer++);
   
  // tmp1 = pgm_read_byte(pointer++);
   
   //display_go_to(char_x,char_y-1);
   
   //display_write_int((tmp1 & 0xFF),2);
   //tmp1 <<=4;
   uint8_t line_y = char_y;

	uint8_t sub_y = char_y -1;
   //char_y = 2;
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(fontwidth)+DISPLAY_OFFSET);col=col+1)
	{
		{
			tmp1 = pgm_read_byte(pointer++);
         //uint8_t offset = 2;
         //tmp1 <<= 8;         // 8 bit nach oben zum voraus: Platz schaffen
         tmp1<<= (8-offset);          // offset nach unten
         uint8_t tmp3 = (tmp1&0xFF00)>>8; // obere 8 bit, 8 bit nach unten, ergibt lo
         uint8_t tmp4 = (tmp1&0x00FF);//>>4; // obere 8 bit,  ergibt hi
         display_go_to(col,char_y);
         display_write_byte(DATA,tmp3);

         display_go_to(col,char_y-1);
         display_write_byte(DATA,tmp4);

         
         //display_write_byte(DATA,(tmp1 & 0xFF00)>>8);
		}
	}
   
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH);
	}
	return;
}






//##############################################################################################
//Ausgabe eines Zeichenkette
//
//##############################################################################################
void display_write_P (const uint8_t *Buffer,...)
{
	va_list ap;
	va_start (ap, Buffer);	
	
	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;
		
	//Ausgabe der Zeichen
    for(;;)
    {
       by = pgm_read_byte(Buffer++);
       if(by==0) break; // end of format string
       
       if (by == '%')
       {
          by = pgm_read_byte(Buffer++);
          if (isdigit(by)>0)
          {
             
             str_null_buffer[0] = by;
             str_null_buffer[1] = '\0';
             move = atoi(str_null_buffer);
             by = pgm_read_byte(Buffer++);
          }
          
          switch (by)
          {
             case 's':
                ptr = va_arg(ap,char *);
                while(*ptr) { display_write_propchar(*ptr++,1); }
                break;
             case 'b':
                Base = 2;
                goto ConversionLoop;
             case 'c':
                //Int to char
                format_flag = va_arg(ap,int);
                display_write_propchar (format_flag++,1);
                break;
             case 'i':
                Base = 10;
                goto ConversionLoop;
             case 'o':
                Base = 8;
                goto ConversionLoop;
             case 'x':
                Base = 16;
                //****************************
             ConversionLoop:
                //****************************
                itoa(va_arg(ap,int),str_buffer,Base);
                int b=0;
                while (str_buffer[b++] != 0){};
                b--;
                if (b<move)
                {
                   move -=b;
                   for (tmp = 0;tmp<move;tmp++)
                   {
                      str_null_buffer[tmp] = '0';
                   }
                   //tmp ++;
                   str_null_buffer[tmp] = '\0';
                   strcat(str_null_buffer,str_buffer);
                   strcpy(str_buffer,str_null_buffer);
                }
                display_write_str ((const unsigned char*)str_buffer,1);
                move =0;
                break;
          }
          
       }	
       else
       {
          display_write_propchar ( by ,1);
       }
    }
	va_end(ap);
}

//##############################################################################################
//Ausgabe eines Strings
//
//##############################################################################################
void display_write_str(const unsigned char *str, uint8_t prop)
{
	while (*str)
	{
		display_write_propchar(*str++,prop);
	}
}



void display_write_inv_str(const unsigned char *str,uint8_t prop)
{
   display_go_to(char_y,char_x);
   display_write_byte(DATA,0xFF);
   
   //char_x ++;
	while (*str)
	{
		display_write_inv_propchar(*str++,prop);
	}
}

//##############################################################################################
//Ausgabe einer Zahl
//
//##############################################################################################
void display_write_int(uint8_t zahl, uint8_t prop)
{
   unsigned char zahlbuffer[10];
   itoa(zahl,(char*)zahlbuffer,10);
   
	//while (*zahlbuffer)
	{
		display_write_str((const unsigned char*)zahlbuffer,prop);
      
	}
}

//##############################################################################################
//Ausgabe einer Dezimal-Zahl mit Nachkommastellen
//
//##############################################################################################

void display_write_dez(uint16_t zahl, uint8_t stellen, uint8_t prop)
{
   // zahl ist folge von ziffern ohne Punkt.
   #define ANZAHLELEMENTE 6
   char string[ANZAHLELEMENTE]={};
   int8_t i;                             // schleifenzähler
   int8_t flag=0;
   string[ANZAHLELEMENTE-1]='\0';                       // String Terminator
   for(i=ANZAHLELEMENTE-2; i>=0; i--)
   {
      if (i==ANZAHLELEMENTE-stellen-2)
      {
         string[i] = '.';
      }
      else
      {
         string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
         zahl /= 10;
      }
   }
   
   char c;
   i=0;
   while ( (c = string[i]) )
   {
      if (c>'0')
      {
         flag=1;
      }
      if (flag )
      {
  
         display_write_propchar(c,prop);
      }
      i++;
   }
}
//##############################################################################################
void display_write_hex(uint8_t zahl, uint8_t prop) // von lcd_puthex
{
   uint8_t dat[3];
   uint8_t i,l,h;     
   dat[2]='\0';                       // String Terminator
   l=(zahl % 16);
   if (l<10)
   dat[1]=l +'0';  
   else
   {
   l%=10;
   dat[1]=l + 'A'; 
   
   }
   zahl /=16;
   h= zahl % 16;
   if (h<10)
   dat[0]=h +'0';  
   else
   {
   h%=10;
   dat[0]=h + 'A'; 
   }

   display_write_str((const unsigned char*)dat,prop);
   
}
//##############################################################################################
void display_write_sec_min(uint16_t zeit, uint8_t prop)
{
   
   unsigned char zeitString[6];
   zeitString[5]='\0';

   uint8_t sekunden = zeit%60;
   
   
   //   sekunden einsetzen
   zeitString[4]=(sekunden % 10) +'0';   //hinterste Stelle
   if (sekunden>9)
   {
      sekunden/=10;
      zeitString[3]=(sekunden % 10) +'0';
   }
   else
   {
      zeitString[3]='0';
   }
    
   zeitString[2]=':';
   
   uint16_t minuten = zeit/60;
   //   Stunden einsetzen
   zeitString[1]=(minuten % 10) +'0'; 
   if (minuten>9)
   {      
      minuten/=10;
      zeitString[0]=(minuten % 10) +'0';
   }
   else
   {
      zeitString[0]='0';
   }

   
   
   
   
   display_write_str((const unsigned char*)zeitString,prop);
   
   
}
//##############################################################################################












//##############################################################################################
//Ausgabe Spannung
//
//##############################################################################################
void display_write_spannung(uint16_t rawspannung, uint8_t prop) // eine Dezimale
{
   uint16_t tempspannung = rawspannung;
   display_write_dez(tempspannung,1,prop);
   display_write_propchar('V',prop);
   
   /*
   uint8_t dezimale = temp%10;
   uint8_t wert = temp/10;
   char tempbuffer[6]={};
  
   {
     // tempbuffer[0] =' ';
   }
   tempbuffer[0] =wert%10+'0';
   tempbuffer[1] ='.';
   
   tempbuffer[2] =dezimale+'0';
   tempbuffer[3] ='V';
   tempbuffer[4] = '\0';
   display_write_str(tempbuffer,1);
*/
   
}


//##############################################################################################
//Ausgabe Minute:Sekunde
//
//##############################################################################################
void display_write_min_sek(uint16_t rawsekunde , uint8_t prop)
{
   uint8_t minute = rawsekunde/60%60;
   uint8_t sekunde = rawsekunde%60;
   uint8_t stunde = rawsekunde/3600;
   
   char tempbuffer[6]={};
   if (stunde)
   {
      char stdbuffer[4]={};
      stdbuffer[0] =stunde/10+'0';
      stdbuffer[1] =stunde%10+'0';
      stdbuffer[2] =':';
      stdbuffer[3] = '\0';
      display_write_str((const unsigned char*)stdbuffer,prop);
   }
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str((const unsigned char*)tempbuffer,prop);
   
}

void display_write_zeit(uint8_t sekunde,uint8_t minute,uint8_t stunde,uint8_t prop)
{
   
   char tempbuffer[6]={};
   if (stunde)
   {
      char stdbuffer[4]={};
      stdbuffer[0] =stunde/10+'0';
      stdbuffer[1] =stunde%10+'0';
      stdbuffer[2] =':';
      stdbuffer[3] = '\0';
      display_write_str((const unsigned char*)stdbuffer,prop);
   }
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str((const unsigned char*)tempbuffer,prop);
   
}

void display_write_stopzeit(uint8_t sekunde,uint8_t minute,uint8_t prop)
{
   
   char tempbuffer[6]={};
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str((const unsigned char*)tempbuffer,prop);
   
}

void display_write_stopzeit_BM(uint8_t sekunde,uint8_t minute)
{
   
   char tempbuffer[2][64]={};
   uint8_t col,page,tmp1,tmp2;
   col=char_x;
   page = char_y-1;
   display_go_to(col,page);
   
   uint8_t index=0, startposition=0, endposition=10, delta=11, breite=12;
   
   // Digit 1 minute zehner
   //PGM_P pointer = (PGM_P)zahlfont12[minute/10];
   //uint16_t* pointer = (uint16_t*)zahlfont12[minute/10];
   for (index=0;index< breite;index++)
   //for (index = 0;index < 10;index++)
   {
      tmp1 = (char)pgm_read_byte(&(zahlfont12[minute/10][2*index])); // byte col, lo
      tempbuffer[0][startposition+index] = tmp1;
      tmp2 = (char)pgm_read_byte(&(zahlfont12[minute/10][2*index+1])); // byte col, lo
      tempbuffer[1][startposition+index] = tmp2;
      
  }
   
   
   // Digit 2 minute einer
   startposition+= delta;
  // char_x +=delta;
   //pointer = (PGM_P)zahlfont12[minute%10];
   
   for (index=0;index<breite;index++)
   {
    tmp1 = (char)pgm_read_byte(&(zahlfont12[minute%10][2*index])); // byte col, lo
    tempbuffer[0][startposition+index] = tmp1;
    tmp2 = (char)pgm_read_byte(&(zahlfont12[minute%10][2*index+1])); // byte col, lo
    tempbuffer[1][startposition+index] = tmp2;
   }
   
   
   startposition+= delta;
   
   // Doppelpunkt
 
   for (index=0;index<4;index++)
{
   tmp1 = (char)pgm_read_byte(&(zahlfont12[10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   
   
   startposition+= 5;
 

   // Digit 3 sekunde zehner
   
   for (index=0;index<breite;index++)
{
   tmp1 = (char)pgm_read_byte(&(zahlfont12[sekunde/10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[sekunde/10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   
   startposition+= delta;
   
    // Digit 4 sekunde zehner
   
   for (index=0;index<breite;index++)
   {
   tmp1 = (char)pgm_read_byte(&(zahlfont12[sekunde%10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[sekunde%10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   endposition = 52; // Ganzen Block zeichnen
   for (index=0;index<endposition;index++)
   {
      display_write_byte(0,tempbuffer[1][index]);
   }
   
   display_go_to(col,page+1);
   
   for (index=0;index<endposition;index++)
   {
      display_write_byte(0,tempbuffer[0][index]);
   }
   
   /*
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str(tempbuffer,prop);
   */
   
}

void display_write_stopzeit_BM1(uint8_t sekunde,uint8_t minute)
{
   
   char tempbuffer[2][48]={};
   uint8_t col,page,tmp1,tmp2;
   
   col=char_x;
   page = char_y-1;
   display_go_to(col,page);
   uint16_t index=0;
   
   // Digit 1 minute zehner
  // PGM_P pointer = (PGM_P)zahlfont12[minute/10];
   //uint16_t* pointer = (uint16_t*)zahlfont12[minute/10];

   for (index=0;index<10;index++)
   {
      //tmp1 = pgm_read_byte(pointer++); // byte col, lo
     tmp1 = (char)pgm_read_byte(&(zahlfont12[0][2*index])); // byte col, lo
      tempbuffer[0][index] = tmp1;
    //  tmp2 = pgm_read_byte(pointer++); // byte col+1, hi
     tmp2 = (char)pgm_read_byte(&(zahlfont12[0][2*index+1])); // byte col, lo
      tempbuffer[1][index] = tmp2;
   }
   /*
   // Digit 2 minute einer
   pointer = (PGM_P)zahlfont12[minute%10];
   
   for (index=7;index<14;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Doppelpunkt
   pointer = (PGM_P)zahlfont12[11];
   
   for (index=14;index<19;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Digit 3 sekunde zehner
   pointer = (PGM_P)zahlfont12[sekunde/10];
   
   for (index=17;index<24;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Digit 4 sekunde zehner
   pointer = (PGM_P)zahlfont12[sekunde%10];
   
   for (index=24;index<31;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   */
   
   for (index=0;index<20;index++)
   {
      display_write_byte(0,tempbuffer[1][index]);
   }
   
   display_go_to(col,page+1);
   
   for (index=0;index<10;index++)
   {
      display_write_byte(0,tempbuffer[0][index]);
   }
   
   /*
    tempbuffer[0] =minute/10+'0';
    tempbuffer[1] =minute%10+'0';
    tempbuffer[2] =':';
    tempbuffer[3] =sekunde/10+'0';
    tempbuffer[4] =sekunde%10+'0';
    tempbuffer[5] = '\0';
    display_write_str(tempbuffer,prop);
    */
   
}



//##############################################################################################
// Ausgabe Pfeil rechts an pos x,y (page)
//##############################################################################################
void display_pfeilvollrechts(uint8_t col, uint8_t page)
{
   char_x = col;
   char_y = page;
   display_write_symbol((const unsigned char*)pfeilvollrechts);
   
}

//##############################################################################################
//Ausgabe eines Symbols
//
//##############################################################################################
void display_write_symbol(const uint8_t*  symbol)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
	const unsigned char* pointer = symbol;
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATA,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}

void display_write_propsymbol(const uint8_t* symbol)
{
	uint8_t col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = symbol;
   uint8_t charsize = 8;
   uint8_t  charbreite =   pgm_read_byte(pointer++);
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATA,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATA,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}


/*
 http://www.mikrocontroller.net/attachment/5130/spi.c
 void write_spi (uint8_t data_out){       //msb first
 uint8_t loop, mask;
 for (loop=0,mask=0x80;loop<8;loop++, mask=mask>>1)
 {sclk=0;
 if (data_out & mask) mosi=1;
 else mosi=0;
 sclk=1;
 }
 sclk=0;
 }

 */


/*
 * File:        dogl.c
 * Project:     Mini Anzeige Modul
 * Author:      Nicolas Meyertˆns
 * Version:     siehe setup.h
 * Web:         www.PIC-Projekte.de

 * Diese Funktion gibt eine Zeichenkette auf dem Bildschirm aus.
 * Mit einer Angabe der Adresse, kann dies an jedem beliebigen Ort auf
 * dem Display stattfinden. Des Weiteren kann zwischen normaler und
 * invertierter Darstellung gew‰hlt werden.
 *
 * page:        Adresse - Zeile (0..7)
 * column:      Adresse - Spalte (0..127)
 * inverse:     Invertierte Darstellung wenn true sonst normal
 * *pChain:     Die Zeichnkette ansich, welche geschrieben werden soll
 */

/*
 * Auswahl einer Adresse. Diese Funktion wird in der Regel nicht vom
 * Anwender selbst aufgerufen, sondern nur durch die Funktionen dieser
 * C-Datei.
 *
 * page:    Adresse - Zeile (0..7)
 * column:  Adresse - Spalte (0..127)
 */
void setAddrDOGL(uint8_t page, uint8_t column)
{
   if( page<8 && column<128 )
   {
      
      display_write_byte(CMD,0xB0 + page);
      display_write_byte(CMD,0x10 + ((column&0xF0)>>4) );
      display_write_byte(CMD,0x00 +  (column&0x0F) );
      
   }
}

void display_write_prop_str(uint8_t page, uint8_t column, uint8_t inverse, const uint8_t *pChain, uint8_t prop)
{
   uint8_t l=0,k;
   //setAddrDOGL(page-1,column);
   // Space schreiben
   /*
   if(inverse)
   {
      display_write_byte(DATA,~propfont6[0][1]);
      column++;
   }
   else
   {
      display_write_byte(DATA,propfont6[0][1]);
      column++;
   }
   */
   while(*pChain)
   {
      display_write_propchar(*pChain++,prop);
      /*
      PGM_P pointer = propfont6[*pChain-32];
      char blank =propfont6[0][1];
      
      
      switch (prop)
      {
         case 2:
         {
            pointer = propfont8[*pChain-32];
         }break;
      }
      //PGM_P pointer = propfont6[*pChain-32];
      uint8_t  fontbreite =   pgm_read_byte(pointer++);
      uint8_t tmp1=0;
      for(k=1; k <= fontbreite; k++)
      {
         if( column > 127)
            break;
         
         display_go_to(column,page-1);
         tmp1 = pgm_read_byte(pointer++);
         column++;
         
         if(inverse)
         {
            display_write_byte(DATA,~tmp1);
         }
         else
         {
            display_write_byte(DATA,tmp1);
         }
      }
      if( column > 127)
      {
         return ;
      }
      
      // Space schreiben
      if(inverse)
      {
         display_write_byte(DATA,~0x00);
         column++;
      }
      else
      {
         display_write_byte(DATA,0x00);
         column++;
      }
      */
      /*
       * Ab dem (126-32). Eintrag in der Font Library folgen die Zeichen
       * des MiniAnzeigeModuls. Diese Eintr‰ge kommen NICHT als String in
       * diese Funktion und haben somit keinen Terminator! Es muss beim
       * Zeichnen eines Zeichens also darauf geachtet werden, dass der
       * Pointer nicht einen Schritt weiter geht sondern direkt die
       * Schleife beendet. Das geschieht hier:
       */
      if( *pChain > 126 )
         break;
      pChain++;
   } // while *char
   
   return ;
}


//##############################################################################################

void r_uitoa_disp(int8_t zahl, uint8_t* string)
{
   uint8_t i;
   
   string[3]='\0';                  // String Terminator
   for(i=3; i>=0; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
}
//##############################################################################################


uint8_t spi_out(uint8_t dataout)
{
   cli();
  // OSZI_B_LO;
   CS_LO; // Chip enable
   uint8_t datain=0xFF;
   uint8_t pos=0;
   SCL_LO; // SCL LO
   uint8_t tempdata=dataout;

   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         //DATA_HI;
         SOFT_SPI_PORT |= (1<<DOG_DATA);

      }
      else
      {
         //DATA_LO;
         SOFT_SPI_PORT &= ~(1<<DOG_DATA);
         //DOG_PORT |= (1<<DOG_DATA);
      }
      tempdata<<= 1;
     // _delay_us(10);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<DOG_SCL);
      //_delay_us(10);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<DOG_SCL);
      
   }
 //  OSZI_B_HI;
   
   CS_HI;// Chip disable
   
   sei();
   return datain;
}


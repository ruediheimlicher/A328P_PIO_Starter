/*
    File:       lcd.c
    Version:    0.1.0
    Date:       Feb. 25, 2006
    
    C header file for HD44780 LCD module using a 74HCT164 serial in, parallel 
    out out shift register to operate the LCD in 8-bit mode.  Example schematic
    usuing a Powertip PC-1202A LCD available at http://www.micahcarrick.com
    
    Uses the avr-libc library for gcc (avr-gcc).
    
    ****************************************************************************
    Copyright (C) 2006 Micah Carrick   <email@micahcarrick.com>
	
	//Use:

        DDRD = 0xF0;                    // PD4:7 digital outputs //
         
        //turn on backlight 
        lcd_backlight(1);      

        // initialize the LCD 
        lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
        
        // write my first name 
        lcd_putc('M'); 
        lcd_putc('i');
        lcd_putc('c');
        lcd_putc('a');
        lcd_putc('h');
        
        // goto line 2 
        lcd_move_cursor(LCD_LINE_2, 0);
        
        // write my last name 
        lcd_putc('C');
        lcd_putc('a');
        lcd_putc('r');
        lcd_putc('r');
        lcd_putc('i');
        lcd_putc('c');
        lcd_putc('k');
        
        return (0);

    ****************************************************************************
*/

#include "lcd.h"
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

extern char* wochentagstring[];
/*
 * Turns the backlight on or off.  The LCD_BACKLIGHT_PIN should be defined as
 * the pin connected to the backlight control of the LCD.
 *
 * Parameters:
 *      backlight_on    0=off, 1=on
*/

void lcddelay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}



void 
lcd_backlight(int backlight_on)
{
        if (backlight_on)
        {
                /* LCD Backlight on */
                LCD_PORT |= _BV(LCD_BACKLIGHT_PIN);                
        } else {
                /*  LCD Backlight off */
                LCD_PORT &= ~_BV(LCD_BACKLIGHT_PIN);
        }
}


void
lcd_initialize(uint8_t set_function, uint8_t set_entry_mode, uint8_t on)
{
        /* 20ms delay while LCD powers on */
        _delay_ms(30);	   
        
        /* Write 0x30 to LCD and wait 5 mS for the instruction to complete */
        lcd_load_byte(0x30);
        lcd_send_cmd();
        _delay_ms(10);
        
        /* Write 0x30 to LCD and wait 160 uS for instruction to complete */
        lcd_load_byte(0x30);
        lcd_send_cmd();
        _delay_us(20);
        
        /* Write 0x30 AGAIN to LCD and wait 160 uS */
        lcd_load_byte(0x30);
        lcd_send_cmd();
        _delay_us(160);
        
        /* Set function and wait 40uS */
        lcd_load_byte(set_function);
        lcd_send_cmd();
        
        /* Turn off the display and wait 40uS */
        lcd_load_byte(LCD_CMD_OFF);    
        lcd_send_cmd();
        
        /* Clear display and wait 1.64mS */
        lcd_load_byte(LCD_CMD_CLEAR);    
        lcd_send_cmd();
        _delay_ms(4);
        
        /* Set entry mode and wait 40uS */
        lcd_load_byte(set_entry_mode);    
        lcd_send_cmd();
        _delay_ms(4);
        /* Turn display back on and wait 40uS */
        lcd_load_byte(on);    
        lcd_send_cmd();
		_delay_ms(40);
};

/*
 * Loads a byte into the shift register (74'164).  Does NOT load into the LCD.
 *
 * Parameters:
 *      out_byte        The byte to load into the '164.
*/
void 
lcd_load_byte(uint8_t out_byte)
{
        /* make sure clock is low */
        LCD_PORT &= ~_BV(LCD_CLOCK_PIN);
        
        int i;
        for(i=0; i<8; i++)
        {
                /* loop through bits */
                
                if (out_byte & 0x80)
                {
                        /* this bit is high */
                        LCD_PORT |=_BV(LCD_RSDS_PIN); 
                }
                else
                {
                        /* this bit is low */
                        LCD_PORT &= ~_BV(LCD_RSDS_PIN);
                }
                out_byte = out_byte << 1;
                
                /* pulse the the shift register clock */
                LCD_PORT |= _BV(LCD_CLOCK_PIN);
                LCD_PORT &= ~_BV(LCD_CLOCK_PIN);
        }
}

/*
 * Loads the byte in the '164 shift register into the LCD as a command. The
 * '164 should already be loaded with the data using lcd_load_byte().
*/
void
lcd_send_cmd(void)
{
        /* Data in '164 is a command, so RS must be low (0) */
        LCD_PORT &= ~_BV(LCD_RSDS_PIN); 
        lcd_strobe_E();
        _delay_us(50);
}

/*
 * Loads the byte in the '164 shift register into the LCD as a character. The
 * '164 should already be loaded with the data using lcd_load_byte().
*/
void
lcd_send_char(void)
{
        /* Data in '164 is a character, so RS must be high (1) */
        LCD_PORT |= _BV(LCD_RSDS_PIN); 
        lcd_strobe_E();
        _delay_us(50);
}

/*
 * Loads the byte into the shift register and then sends it to the LCD as a char
 * Parameters:
 *      c               The byte (character) to display
*/
void 
lcd_putc(const char c)
{
        lcd_load_byte(c);
        lcd_send_char();
}

/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */



void lcd_putint(uint8_t zahl)
{
	//char string[5];
	char string[4];
	int8_t i;                             // schleifenz�hler
	
	//  string[4]='\0';                       // String Terminator
	string[3]='\0';                       // String Terminator
	for(i=2; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	lcd_puts(string);
}

void lcd_putint2(uint8_t zahl)	//zweistellige Zahl
{

	char string[3];
	int8_t i;								// Schleifenz�hler
	zahl%=100;								// 2 hintere Stelle
	//  string[4]='\0';                     // String Terminator
	string[2]='\0';							// String Terminator
	for(i=1; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	lcd_puts(string);
}

void lcd_putint1(uint8_t zahl)	//einstellige Zahl
{
	//char string[5];
	char string[2];
	zahl%=10;								//  hinterste Stelle
	string[1]='\0';							// String Terminator
	string[0]=zahl +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
	lcd_puts(string);
}


void lcd_puthex(uint8_t zahl)
{
	//char string[5];
	char string[3];
	uint8_t i,l,h;                             // schleifenz�hler
	
	string[2]='\0';                       // String Terminator
	l=(zahl % 16);
	if (l<10)
	string[1]=l +'0';  
	else
	{
	l%=10;
	string[1]=l + 'A'; 
	
	}
	zahl /=16;
	h= zahl % 16;
	if (h<10)
	string[0]=h +'0';  
	else
	{
	h%=10;
	string[0]=h + 'A'; 
	}
	
	
	lcd_puts(string);
}




/*
 * Strobes the E signal on the LCD to "accept" the byte in the '164.  The RS
 * line determines wheter the byte is a character or a command.
*/
void
lcd_strobe_E(void)
{
        /* strobe E signal */
        LCD_PORT |= _BV(LCD_ENABLE_PIN);
        _delay_us(550); 
	//	lcddelay_ms(100);
        LCD_PORT &= ~_BV(LCD_ENABLE_PIN);
}

/*
 * Moves the cursor to the home position.
*/
void
lcd_cursor_home(void)
{
        lcd_load_byte(LCD_CMD_HOME);
        lcd_send_cmd();
}

/*
 * Moves the cursor to the specified position.
 * Parameters:
 *      line            Line (row)
 *      pos             Position on that line (column)
*/
void
lcd_move_cursor(uint8_t line, uint8_t pos)
{
//fleury: lcd_command((1<<LCD_DDRAM)+addressCounter)
        lcd_load_byte(line+pos);
        lcd_send_cmd();
}

/*
 * Moves the cursor a number of spaces to the right
 * Parameters:
 *      spaces          Number of spaces to move
*/
void
lcd_inc_cursor(uint8_t spaces)
{
        while (spaces--)
        {
                lcd_load_byte(LCD_CMD_CURSOR_RIGHT);
                lcd_send_cmd();    
        }
}

/*
 * Moves the cursor a number of spaces to the left
 * Parameters:
 *      spaces          Number of spaces to move
*/
void
lcd_dec_cursor(uint8_t spaces)
{
        while (spaces--)
        {
                lcd_load_byte(LCD_CMD_CURSOR_LEFT);
                lcd_send_cmd();    
        }
}

/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
switch (y)
{
    case 0:
        lcd_load_byte((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		lcd_send_cmd();
		break;
   case 1:
		lcd_load_byte((1<<LCD_DDRAM)+LCD_START_LINE2+x);
		lcd_send_cmd();
		break;
	case 2:
		lcd_load_byte((1<<LCD_DDRAM)+LCD_START_LINE3+x);
		lcd_send_cmd();
		break;
	case 3:
		lcd_load_byte((1<<LCD_DDRAM)+LCD_START_LINE4+x);
		lcd_send_cmd();
		break;
	

}//switch


}/* lcd_gotoxy */


// Linie loeschen
void lcd_clr_line(uint8_t Linie)
{
	lcd_gotoxy(0,Linie);
	uint8_t i=0;
	for (i=0;i<LCD_DISP_LENGTH;i++)
	{
		lcd_putc(' ');
	}
	lcd_gotoxy(0,Linie);
	
}	// Linie loeschen


// Teil einer Linie loeschen
void lcd_clr_part(uint8_t Linie, uint8_t von, uint8_t bis)
{
	if (bis>LCD_DISP_LENGTH)
		bis=LCD_DISP_LENGTH;
		
	lcd_gotoxy(von,Linie);
	uint8_t i=0;
	for (i=von;i<bis;i++)
	{
		lcd_putc(' ');
	}
	lcd_gotoxy(von,Linie);
	
}	// Teil einer Linie loeschen





// Display loeschen
void lcd_cls()   
{
	lcd_load_byte(0x01);
	lcd_send_cmd();
//	lcd_write(0x02,0);   	// B 0000 0010 => Display loeschen
	lcddelay_ms(4);			// dauert eine Weile, Wert ausprobiert (2)

//	lcd_write(0x01,0);   	// B 0000 0001 => Cursor Home
	lcd_load_byte(0x02);
	lcd_send_cmd();
	
	lcddelay_ms(4);			// dauert eine Weile, Wert ausprobiert (2)
}




/*
 
Funktion zur Umwandlung einer vorzeichenlosen 32 Bit Zahl in einen String
 
*/
 
void r_uitoa(uint32_t zahl, char* string) {
  int8_t i;                             // schleifenz�hler
 
  string[10]='\0';                       // String Terminator
  for(i=9; i>=0; i--) {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}


/*
 
Funktion zur Umwandlung einer vorzeichenbehafteten 32 Bit Zahl in einen String
 
*/
 
void r_itoa(int32_t zahl, char* string) 
{
  uint8_t i;
 
  string[11]='\0';                  // String Terminator
  if( zahl < 0 ) {                  // ist die Zahl negativ?
    string[0] = '-';              
    zahl = -zahl;
  }
  else string[0] = ' ';             // Zahl ist positiv
 
  for(i=10; i>=1; i--) {
    string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}

 
void r_itoa16(int16_t zahl, char* string) 
{
  uint8_t i;
 
  string[7]='\0';                  // String Terminator
  if( zahl < 0 ) {                  // ist die Zahl negativ?
    string[0] = '-';              
    zahl = -zahl;
  }
  else string[0] = ' ';             // Zahl ist positiv
 
  for(i=6; i>=1; i--) {
    string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}



/*
 
Funktion zur Anzeige einer 32 Bit Zahl im Stringformat
auf einem LCD mit HD44780 Controller
Quelle: www.mikrocontroller.net/articles/Festkommaarithmetik
Parameter:
 
char* string  : Zeiger auf String, welcher mit my_itoa() erzeugt wurde
uint8_t start : Offset im String, ab der die Zahl ausgegeben werden soll,
                das ist notwenig wenn Zahlen mit begrenztem Zahlenbereich
                ausgegeben werden sollen
                Vorzeichenlose Zahlen      : 0..10
                Vorzeichenbehaftete zahlen : 1..11
uint8_t komma : Offset im String, zeigt auf die Stelle an welcher das virtuelle
                Komma steht (erste Nachkommastelle)
                komma muss immer gr�sser oder gleich start sein !
 
uint8_t frac  : Anzahl der Nachkommastellen
 
*/
 
void lcd_put_frac(char* string, uint8_t start, uint8_t komma, uint8_t frac) 
{
 
  uint8_t i;            // Z�hler
  uint8_t flag=0;       // Merker f�r f�hrende Nullen
 
  // Vorzeichen ausgeben  
  if (string[0]=='-') lcd_putc('-'); else lcd_putc(' ');
 
  // Vorkommastellen ohne f�hrende Nullen ausgeben
  for(i=start; i;i--) {
    if (flag==1 || string[i]!='0') {
      lcd_putc(string[i]);
      flag = 1;
    }
    else lcd_putc(' ');         // Leerzeichen
  }
 
  lcd_putc('.');                // Komma ausgeben
 
  // Nachkommastellen ausgeben
  for(; i<(komma+frac); i++) lcd_putc(string[i]);
 
}

void lcd_put_zeit(uint8_t dieMinuten, uint8_t dieStunden)
{
	//							13:15
	int8_t minuten=dieMinuten; 
	uint8_t stunden=dieStunden;
	if (stunden< 10)
	{
		//	lcd_putc(' ');
	}
	
	char zeitString[6];
	zeitString[5]='\0';
	
	//	Minuten einsetzen
	zeitString[4]=(minuten % 10) +'0';	//hinterste Stelle
	if (minuten>9)
	{
		minuten/=10;
		zeitString[3]=(minuten % 10) +'0';
	}
	else
	{
		zeitString[3]='0';
	}
	 
	zeitString[2]=':';
	
	//	Stunden einsetzen
	zeitString[1]=(stunden % 10) +'0'; 
	if (stunden>9)
	{		
		stunden/=10;
		zeitString[0]=(stunden % 10) +'0';
	}
	else
	{
		zeitString[0]='0';
	}
	 
	
	lcd_puts(zeitString);
}

void lcd_put_wochentag(uint8_t wd)
{
	//char* wochentag[] = {"MO","DI","MI","DO","FR","SA","SO"};
	lcd_puts(wochentagstring[wd]);	// Array wochentag ist null-basiert
}

void lcd_put_temperatur(uint16_t temperatur)
{
		char buffer[8]={};
		uint16_t temp=(temperatur-127)*5;
//		uint16_t temp=temperatur;
		
//		itoa(temp, buffer,10);
		r_itoa16(temp,buffer);
		lcd_puts(buffer);
//		lcd_putc(' * ');
		
		char outstring[8]={};

		outstring[7]='\0';
		outstring[6]=0xDF;
		outstring[5]=buffer[6];
		outstring[4]='.';
		outstring[3]=buffer[5];
		if (abs(temp)<100)
		{
		outstring[2]=' ';
		outstring[1]=' ';
		}
		else if (abs(temp)<1000)
		{
		outstring[2]=buffer[4];
		outstring[1]=' ';
		
		}
		else
		{
		outstring[2]=buffer[4];
		outstring[1]=buffer[3];

		}
		
		outstring[0]=buffer[0];
/*
		if (temp<100)
		{
		lcd_putc(' ');
		}
		if (temp<10)
		{
		lcd_putc(' ');
		}
	*/	
		lcd_puts(outstring);

}



void lcd_put_tempbis99(uint16_t temperatur)
{
		char buffer[7]={};
		//uint16_t temp=(temperatur-127)*5;
		uint16_t temp=temperatur*5;
		
//		itoa(temp, buffer,10);
		r_itoa16(temp,buffer);
//		lcd_puts(buffer);
//		lcd_putc(' * ');
		
		char outstring[7]={};

		outstring[6]='\0';
		outstring[5]=0xDF;
		outstring[4]=buffer[6];
		outstring[3]='.';
		outstring[2]=buffer[5];
		if (abs(temp)<100)
		{
		outstring[1]=' ';
		
		}
		else 
		{
		outstring[1]=buffer[4];
				
		}		
		outstring[0]=buffer[0];
/*
		if (temp<100)
		{
		lcd_putc(' ');
		}
		if (temp<10)
		{
		lcd_putc(' ');
		}
	*/	
		lcd_puts(outstring);

}


void lcd_put_tempAbMinus20(uint16_t temperatur)
{

		char buffer[8]={};
		int16_t temp=(temperatur)*5;
		temp -=200;
		char Vorzeichen=' ';
		if (temp < 0)
		{
			Vorzeichen='-';
		}

		r_itoa16(temp,buffer);
//		lcd_puts(buffer);
//		lcd_putc(' * ');
		
		char outstring[7]={};

		outstring[6]='\0';
		//outstring[5]=0xDF; // Grad-Zeichen
		outstring[5]=' ';
		outstring[4]=buffer[6];
		outstring[3]='.';
		outstring[2]=buffer[5];
		if (abs(temp)<100)
		{
		outstring[1]=Vorzeichen;
		outstring[0]=' ';
		}
		else
		{
		outstring[1]=buffer[4];
		outstring[0]=Vorzeichen;
		}
		lcd_puts(outstring);
		lcddelay_ms(2);
}

/*************************************************************************/

void lcd_CGRAMInit_A(void)
{
	uint8_t i=0;
        lcd_load_byte(0x48);
        lcd_send_cmd();
		
		for (i=0;i<6;i++)	
		//					Kolonne 1 ganz links		char 1
		{
		
		lcd_load_byte(0x10);
		lcd_send_char();
		
		//		lcd_putc(0x10);
		//lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		for (i=0;i<6;i++)
		//					Kolonne 1+2					char 2
		{
		lcd_load_byte(0x18);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	
		//					Kolonne 1,2,3				char 3
		{
		lcd_load_byte(0x1A);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)
			//					Kolonne 1,2,3,4				char 4
		{
		lcd_load_byte(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	
		//					Kolonne 2,3,4				char 5
		{
		lcd_load_byte(0x0B);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	
		//					Kolonne 3,4					char 6
		{
		lcd_load_byte(0x03);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 4					char 7
		{
		lcd_load_byte(0x01);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		for (i=0;i<6;i++)	
		//					OFF							char 8
		{
		lcd_load_byte(0x00);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		lcd_load_byte(0x80);
        lcd_send_cmd();


}


void lcd_CGRAMInit_Mode(void)
{
	uint8_t i=0;
        lcd_load_byte(0x48);
        lcd_send_cmd();
		
		for (i=0;i<3;i++)	//				beide h	Tag voll, Nacht off			- -	char 1
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x00);
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		for (i=0;i<3;i++)	//				beide h	Tag voll, 2. h Nacht red			- -	char 2
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x01+i%2);
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<3;i++)	//				beide h	Tag voll, 1. h Nacht red			- -	char 3
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x08*((i%2)*2));	//0x08, 0x10
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();

		
		for (i=0;i<3;i++)	//				beide h	Tag voll,  Nacht red			- -	char 4
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x09*((i%2)*2));	//0x09, 0x12
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();

		
		for (i=0;i<3;i++)	//				beide h	Tag voll,  1. h Nacht red, 2. h voll			- -	char 5
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x0B+(i%2*8));	//0x0B, 0x13
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();


		for (i=0;i<3;i++)	//				beide h	Tag voll,  1. h Nacht voll, 2. h red			- -	char 6
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x19+(i%2));	//0x0x19, 0x1A
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();

		for (i=0;i<3;i++)	//				beide h	Tag voll, Nacht voll						- -	char 7
		{
		lcd_putc(0x1B);
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x1B);	//
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();

		for (i=0;i<3;i++)	//				beide h	Tag red, Nacht off						- -	char 7
		{
		lcd_putc(0x09*((i%2)*2));	//0x09, 0x12
		lcd_send_char();
		}
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		for (i=3;i<6;i++)	//					
		{
		lcd_putc(0x00);	//
		lcd_send_char();
		}

		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();





		lcd_load_byte(0x80);
        lcd_send_cmd();


}


void lcd_CGRAMInit_Titel(void)
{
	uint8_t i=0;
	lcd_load_byte(0x48);
	lcd_send_cmd();
	
	//													Strich senkrecht mitte				char 1
	for (i=0;i<8;i++)	
	{
		lcd_load_byte(0x84);//	mittl Pixel 
		lcd_send_char();
	}
	
	//													Ecke links oben						char 2
	for (i=0;i<3;i++)	
	{
		lcd_load_byte(0x00);	//	leer
		lcd_send_char();
	}

	lcd_load_byte(0x87);	//	obere Linie
	lcd_send_char();
	
	for (i=0;i<4;i++)	
	{
		lcd_load_byte(0x84);	//	mittl. Pixel
		lcd_send_char();
	}
	
	//													Ecke links unten					char 3
	for (i=0;i<3;i++)	
	{
		lcd_load_byte(0x84);	//	mittl Pixel 
		lcd_send_char();
	}
	lcd_load_byte(0x87);	//	untere Linie
	lcd_send_char();

	for (i=0;i<4;i++)	
	{
		lcd_load_byte(0x00);	//	leer
		lcd_send_char();
	}
	
	
	
	//													Strich oben							char 4
	lcd_load_byte(0x9F);	//	obere Linie
	lcd_send_char();
	
	for (i=0;i<7;i++)			
	{
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
	}
	
	//													Ecke rechts oben					char 5
		for (i=0;i<3;i++)	
	{
		lcd_load_byte(0x00);	//	leer
		lcd_send_char();
	}
	lcd_load_byte(0x9C);	//	obere Linie
	lcd_send_char();
	
	for (i=0;i<4;i++)	
	{
		lcd_load_byte(0x84); // Pixel mitte
		lcd_send_char();
	}
	
	//													Ecke rechts unten					char 6
	for (i=0;i<3;i++)			
	{
		lcd_load_byte(0x84); // Pixel ganz rechts
		lcd_send_char();
	}
	lcd_load_byte(0x9C);	//	untere Linie
	lcd_send_char();

	for (i=0;i<4;i++)	
	{
		lcd_load_byte(0x00);	//	leer
		lcd_send_char();
	}

	
	//													Pfeil nach oben						char 7
	lcd_load_byte(0x00);	//	obere Linie
	lcd_send_char();
	lcd_load_byte(0x00);	//	leer
	lcd_send_char();
	lcd_load_byte(0x84);	//	mittl. Pixel
	lcd_send_char();
	lcd_load_byte(0x8E);	//	3 mittl. Pixel
	lcd_send_char();
	lcd_load_byte(0x9F);	//	5 Pixel
	lcd_send_char();	
	
	//	3 freie Linien
	for (i=0;i<2;i++)	
	{
		lcd_load_byte(0x00);	//	freie Linie
		lcd_send_char();
	}
	
	//													Pfeil nach unten					char 8
	//	3 freie Linien
	for (i=0;i<3;i++)	
	{
	lcd_load_byte(0x00);	//	freie Linie
	lcd_send_char();
	}
	lcd_load_byte(0x9F);	//	5 Pixel
	lcd_send_char();	
	lcd_load_byte(0x8E);	//	3 mittl. Pixel
	lcd_send_char();
	lcd_load_byte(0x84);	//	mittl Pixel
	lcd_send_char();	

	lcd_load_byte(0x00);	//	freie Linie
	lcd_send_char();
	lcd_load_byte(0x00);	//	freie Linie
	lcd_send_char();
	lcd_load_byte(0x00);	//	freie Linie
	lcd_send_char();
	
	
	
	
	// Abschliessen
	lcd_load_byte(0x80);
	lcd_send_cmd();
			
				
}

/*
void CGRAMInit_B(void)
{
	uint8_t i=0;
        lcd_load_byte(0x48);
        lcd_send_cmd();
		
		for (i=0;i<6;i++)	//					Kolonne 2			_| __	char 1
		{
		lcd_putc(0x08);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		for (i=0;i<6;i++)	//					Kolonne 2, 3		_| |_		char 2
		{
		lcd_load_byte(0x0A);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 3			__ |_		char 3
		{
		lcd_load_byte(0x02);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 1,3,4		|_ ||		char 4
		{
		lcd_load_byte(0x13);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 1,2,4		|| _|		char 5
		{
		lcd_load_byte(0x19);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 1, 4		|_ _|		char 6
		{
		lcd_load_byte(0x11);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		
		for (i=0;i<6;i++)	//					Kolonne 2, 4		_| _|			char 7
		{
		lcd_load_byte(0x09);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		for (i=0;i<6;i++)	//					Kolonne 1, 3		|_ |_				char 8
		{
		lcd_load_byte(0x12);
		lcd_send_char();
		}
		lcd_load_byte(0x1B);//OFF-Linie
		lcd_send_char();
		lcd_load_byte(0x00);//freie Linie
		lcd_send_char();
		
		lcd_load_byte(0x80);
        lcd_send_cmd();


}
*/


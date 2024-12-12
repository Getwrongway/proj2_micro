/*
                        ATENÇÃO

Este ficheiro utiliza o PORT B para envio de dados para o LCD
Caso utilize um PIC de 28pinos na placa de desenvolvimento, chante o PORT D com o PORT B 


*/
//#include <p18f4520.h>  //comment if already inlcluded
#include <stdlib.h>
#include <delays.h>  //comment if already inlcluded

// DATA_PORT_XC8 defines the port to which the LCD data lines are connected
#define DATA_PORT_XC8      PORTD
#define TRIS_DATA_PORT_XC8 TRISD

#define RW_PIN_XC8   LATDbits.LATD5   // PORT for RW
#define TRIS_RW_XC8  DDRDbits.RD5    // TRIS for RW
#define RS_PIN_XC8   LATDbits.LATD4   // PORT for RS
#define TRIS_RS_XC8  DDRDbits.RD4    // TRIS for RS
#define E_PIN_XC8    LATDbits.LATD6   // PORT for E
#define TRIS_E_XC8   DDRDbits.RD6    // TRIS for E

#define E7_PIN_XC8    LATDbits.LATD7   // PORT for ?
#define TRIS_E7_XC8   DDRDbits.RD7    // TRIS for ?

#define LN_INPUT  TRIS_DATA_PORT_XC8|=0x0f  // make low nibble input
#define LN_OUTPUT TRIS_DATA_PORT_XC8&=0xf0  // make low nibble output

/* Display ON/OFF Control defines */
#define DISPLAY_ON  0b00001111  
#define DISPLAY_OFF 0b00001011  
#define CURSOR_ON   0b00001111  
#define CURSOR_OFF  0b00001101  
#define BLINK_ON    0b00001111  
#define BLINK_OFF   0b00001110  


// Deve-se ter em conta a velocidade de relogio utilizada, no caso 4MHz.
// Tendo tambem em conta que um ciclo de instrucção são 4 ciclos de relogio

/**********************************************************************/
void delay_1us (void)  
{   
	Nop ();
	return;
}
/**********************************************************************/
void delay_20ms (void)
{
	Delay1KTCYx(20); 
	return;
}
/**********************************************************************/
void delay_5ms (void)
{
	Delay1KTCYx(5); 
	return;
}
/**********************************************************************/


void clock_data (void)
{
   delay_1us ();                   // delay ~ 1 microsecond
   E_PIN_XC8 = 1;                      // set LCD enable pin HI
   delay_1us ();                   // delay ~ 1 microsecond
   E_PIN_XC8 = 0;                      // set LCD enable pin LO
}
/**********************************************************************/
unsigned char LCD_busy (void)
{
   unsigned char port_data, port_busypin;
 
   LN_INPUT; 		       // make low nibble input
   RW_PIN_XC8 = 1;             // Set the control bits for read
   RS_PIN_XC8 = 0;
   delay_1us ();
   E_PIN_XC8 = 1;              // Clock in the command
   delay_1us ();
   port_data = DATA_PORT_XC8;
   port_busypin = port_data & 0b00001000; // 0x08;
   E_PIN_XC8 = 0;              // Reset clock line
   delay_1us ();
   E_PIN_XC8 = 1;              // Clock in other nibble
   delay_1us ();
   E_PIN_XC8 = 0;
   RW_PIN_XC8 = 0;             // Reset control line
   LN_OUTPUT;  			   // make low nibble output
   return port_busypin;              
}
/**********************************************************************/
void clock_nibble_out (unsigned char data) // to low nibble of port
{
   DATA_PORT_XC8 &= 0xf0;      // clear low nibble of port
   data      &= 0x0f;      // zero any bits in upper nibble of data
   DATA_PORT_XC8 |= data;      // set data to port
   delay_1us ();
   E_PIN_XC8 = 1;              // Clock the data out
   delay_1us ();
   E_PIN_XC8 = 0;
}
/**********************************************************************/
void LCD_command (unsigned char cmd)  // 
{
   LN_OUTPUT;              // low nibble output
   RW_PIN_XC8 = 0;             // Set control signals for command
   RS_PIN_XC8 = 0;
   clock_nibble_out (cmd >> 4); // shift upper nibble to lower nibble
   clock_nibble_out (cmd);      // this outputs lower nibble
}
/**********************************************************************/
void LCD_Clear (void)
{
   while (LCD_busy ());              // Wait if LCD busy
   LCD_command (0x01);             // Clear display - takes ~16 ms
   delay_20ms ();
   LCD_command (DISPLAY_ON & CURSOR_OFF & BLINK_OFF);   
}
/**********************************************************************/
void LCD_init (void)
{
   // LCD will be used in 4-bit mode. 
   // clock_nibble_out uses lower nibble of data port
   // LCD defaults on power-up to 8-bit mode
   
   TRIS_RW_XC8 = 0;                    // set RW pin for output
   TRIS_RS_XC8 = 0;                    // set RS pin for output
   TRIS_E_XC8  = 0;                    // set  E pin for output
   RW_PIN_XC8  = 0;                    // R/W pin made low
   RS_PIN_XC8  = 0;                    // Register select pin made low
   E_PIN_XC8   = 0;                    // Clock pin made low
   
   E7_PIN_XC8=1;
   TRIS_E7_XC8 = 0;
   E7_PIN_XC8=1;

   delay_20ms (); // Delay 15ms min to allow for LCD power-on reset  # 1
       
   LN_OUTPUT;    // set data port low nibble for output

   clock_nibble_out (0b0011);        // "home"                       # 2
   delay_5ms ();  // Delay for at least 4.1ms
   clock_nibble_out (0b0011);        // "home"                       # 4
   delay_5ms ();       // Delay for at least 100us                   # 5

   clock_nibble_out (0b0011);        // "home"                       # 6
   delay_5ms ();       // Delay for at least 100us
   clock_nibble_out (0b0010);   // set LCD interface to 4-bit        # 7
   delay_5ms ();

   clock_nibble_out (0b0010);   // set LCD interface to 4-bit
   clock_nibble_out (0b1000);   // 1 = 2-lines, 0 = 5x7, 00 = xx
   LCD_Clear ();
}
/**********************************************************************/
void LCD_WriteChar (char data)
{
   LN_OUTPUT;  // low nibble output
   RW_PIN_XC8 = 0;                     // Set control signals for data
   RS_PIN_XC8 = 1;
   clock_nibble_out (data >> 4); // shift upper nibble to lower nibble
   clock_nibble_out (data);      // this outputs lower nibble
   LN_INPUT;
}
/**********************************************************************/
void LCD_display (unsigned char line, unsigned char position,const char *text)
{
   int i = 0;
   unsigned char address;

   address = position - 1;
   if (line == (unsigned)2) address += 0x40;
   address += 0b10000000;
   while (LCD_busy ()); 
   LCD_command (address);
   while (text[i])                  // write data to LCD up to null
      {
      while (LCD_busy ());      // wait while LCD is busy
      LCD_WriteChar (text[i]); // write character to LCD
      i++; // step to next character
      }
}
/**********************************************************************/
void LCD_blink_position (unsigned char line, unsigned char position)
{
   unsigned char address;

   address = position - 1;
   if (line == (unsigned) 2) address += 0x40;
   address += 0b10000000;
   while (LCD_busy ()); 
   LCD_command (address);
   while (LCD_busy ()); 
   LCD_command (DISPLAY_ON & CURSOR_ON & BLINK_ON);
}
/**********************************************************************/
void LCD_normal (void)
{
   while (LCD_busy ()); 
   LCD_command (DISPLAY_ON & CURSOR_OFF & BLINK_OFF);
}
/*                                 END                                */
/**********************************************************************/

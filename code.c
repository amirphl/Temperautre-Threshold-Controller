

#include <avr\io.h>
#include <avr\interrupt.h>
#include <util\delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#define degree_sysmbol 0xdf
/* 
   LCD16x2 4 bit ATmega16 interface
   http://www.electronicwings.com
*/

#define LCD_Dir DDRA					/* Define LCD data port direction */
#define LCD_Port PORTA					/* Define LCD data port */
#define RS PA1							/* Define Register Select (data reg./command reg.) signal pin */
#define EN PA3 							/* Define Enable signal pin */


volatile int currentRow = 1;
volatile char dogment[10];
volatile int column = 1;
volatile int overflowCounter = 0;
volatile int last_temperature;
volatile int threshold = 25;

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);				/* RS=0, command reg. */
	LCD_Port |= (1<<EN);				/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}


void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Port |= (1<<RS);				/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Init (void)					/* LCD Initialize function */
{
	LCD_Dir = 0b11111110;						/* Make LCD command port direction as o/p */
	_delay_ms(20);						/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x33);
	LCD_Command(0x32);		    		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	LCD_Command(0x0c);              	/* Display on cursor off*/
	LCD_Command(0x06);              	/* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}


void LCD_String (char *str)				/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)				/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);		/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);		/* Command of first row and required position<16 */
	LCD_String(str);					/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);					/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}
 

void ADC_Init(){										
	//DDRA = 0x00;	        /* Make ADC port as input */
	ADCSRA = 0x87;          /* Enable ADC, with freq/128  */
	ADMUX = 0x40;           /* Vref: Avcc, ADC channel: 0 */
}


int ADC_Read(char channel)							
{
	ADMUX = 0x40 | (channel & 0x07);   /* set input channel to read */
	ADCSRA |= (1<<ADSC);               /* Start ADC conversion */
	while (!(ADCSRA & (1<<ADIF))) _delay_ms(1);     /* Wait until end of conversion by polling ADC interrupt flag */
	ADCSRA |= (1<<ADIF);               /* Clear interrupt flag */
	_delay_ms(1);                      /* Wait a little bit */
	return ADCW;                       /* Return ADC word */
}

void keyfind(){
	PORTC = 0b11101111;
	if ((PINC & 0b00000001) == 0b00000000){
		PORTB = ~(0b00111111);
		//LCD_Char('*');
		threshold = 30;
	}
	if ((PINC & 0b00000010) == 0b00000000){
		PORTB = ~(0b01011011);
		//LCD_Char('2');
		threshold = 14;
	}
	if ((PINC & 0b00000100) == 0b00000000){
		PORTB = ~(0b01001111);
		//LCD_Char('3');
		threshold = 16;
	}

	PORTC = 0b11011111;
	if ((PINC & 0b00000001) == 0b00000000){
		PORTB = ~(0b00000110);
		//LCD_Char('1');
		threshold = 12;
	}
	if ((PINC & 0b00000010) == 0b00000000){
		PORTB = ~(0b01101101);
		//LCD_Char('5');
		threshold = 20;
	}
	if ((PINC & 0b00000100) == 0b00000000){
		PORTB = ~(0b01111101);
		//LCD_Char('6');
		threshold = 22;
	}

	PORTC = 0b10111111;
	if ((PINC & 0b00000001) == 0b00000000){
		PORTB = ~(0b01100110);
		//LCD_Char('4');
		threshold = 18;
	}
	if ((PINC & 0b00000010 )== 0b00000000){
		PORTB = ~(0b01111111);
		//LCD_Char('8');
		threshold = 26;
		
	}
	if ((PINC & 0b00000100) == 0b00000000){
		PORTB = ~(0b01101111);
		//LCD_Char('9');
		threshold = 28;
	}

	PORTC = 0b01111111;
	if ((PINC & 0b00000001) == 0b00000000){
		PORTB = ~(0b00000111);
		//LCD_Char('7');
		threshold = 24;
	}
	if ((PINC & 0b00000010) == 0b00000000){
		PORTB = ~(0b00111111);
		//LCD_Char('0');
		threshold = 32;
	}
	if ((PINC & 0b00000100) == 0b00000000){
		PORTB = ~(0b00111111);
		//LCD_Char('#');
		threshold = 34;
	}
	PORTC = 0b00001111 ;
}


ISR(INT0_vect){
	PORTD |= (1<<PD6);
	keyfind();
	_delay_ms(200);
	PORTD &= ~(1<<PD6);
}


int main()
{
	char Temperature[20];
	memset(Temperature,0,20);
	
	TCNT0 = 0;
	OCR0 = 255;
	//ctc
	TCCR0 |=(1<<CS02)|(0<<CS01)|(0<<CS00)|(1<<WGM01);

	sei();
	DDRA = 0b00000000;
	DDRD  = 0b01110011;
	PORTD |= (1<<PD2);
	GICR |= (1<<INT0);
	MCUCR |= (1<<ISC01);
	DDRB = 0b11111111;
	DDRC = 0b11110000;
	PORTC = 0b00001111;
	
	LCD_Init();
	ADC_Init();

	while(1)
	{
		last_temperature = (int)(ADC_Read(0) * 0.488);
		if(threshold < last_temperature){
			TIMSK |=(1<<OCIE0);
			//PORTD |=(1 << PD4);
		}else{
			TIMSK &= ~(1<<OCIE0);
			//PORTD &= ~(1 << PD4);
		}
	   sprintf(Temperature,"Thresh: %d", threshold);/* convert integer value to ASCII string */
	   //itoa(threshold, Temperature, 10);
	   LCD_String_xy(0 , 0 , Temperature);
	   sprintf(Temperature,"Temperature: %d", last_temperature);/* convert integer value to ASCII string */
	   //itoa(last_temperature, Temperature, 10);
	   LCD_String_xy(1 , 0 , Temperature);
	   _delay_ms(3000);
	}

	return 0;
}

//ISR(TIMER0_OVF_vect)
ISR(TIMER0_COMP_vect)
{
    overflowCounter++;
	if(overflowCounter == 96){
		if(last_temperature > threshold){
			PORTD |= (1 << PD4);
		}else{
			PORTD &= ~(1 << PD4);
		}
		if(PORTD & (1 << PD5)){
			PORTD &= ~(1 << PD5);
		}else{
			PORTD |= (1 << PD5);
		}
		overflowCounter = 0;
	}
}

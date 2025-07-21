/*
 * USART0.h
 *
 * Created: 01/04/2020 10:56:47 a.m.
 *  Author: Gaby Morales
 */ 


#ifndef USART0_H_
#define USART0_H_

/********************** Declaración de funciones ****************************/

void usart_init(); //prototipo de función para iniciar el USART AVR
void send_char(char Data);
void send_string(char* cadena);
void envia_2ch(char h1,char h2);
void envia_3ch(char h1,char h2,char h3);
char get_char();


/******************** Desarrollo de las funciones ***************************/

void usart_init(){			//función para iniciar el módulo USART
	UCSR0B=0b00011000;		//transmisión y recepción habilitados a 8 bits
	UCSR0C=0b00000110;		//asíncrono, sin paridad, 1 bit de parada a 8 bits
	UBRR0=7;				//7 para 115200, 15 para 57600 baudios configuración de velocidad
//	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);		//8 bits para el dato.
//	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) ;	//Habilita recepción y transmisión e interrupción por recepción
}

void send_char(char Data){
	while(!(UCSR0A&(1<<5)));	// mientras el registro UDR0 esté lleno espera
	UDR0 = Data;				// Transmite dato
}


void send_string(char* cadena){		//cadena de caracteres de tipo char
	while(*cadena !=0x00){			//mientras el último valor de la cadena sea diferente a el caracter nulo
		send_char(*cadena);	//transmite los caracteres de cadena
		cadena++;					//incrementa la ubicación de los caracteres en cadena
	}
}

void envia_2ch(char h1,char h2)
{
	send_char(h1);
	send_char(h2);
}

void envia_3ch(char h1,char h2,char h3)
{
	send_char(h1);
	send_char(h2);
	send_char(h3);
	send_char('\n');
}

char get_char()
{
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}


#endif /* USART0_H_ */
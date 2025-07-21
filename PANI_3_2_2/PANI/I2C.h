/*
 * I2C.h
 *
 * Created: 30/11/2018 13:11:03
 *  Author: F1
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <util/twi.h>

void i2c_inicio();
uint8_t i2c_inicia_com();
void i2c_detener();
uint8_t i2c_envia_dato(unsigned char dato);
uint8_t i2c_read_ack ();
uint8_t i2c_read_nack ();


void i2c_inicio()	//Inicializa módulo TWI	
{
	TWBR=10;		//Para vel 400kHz,Fosc 14.74568MHz, prescaler de 1
	TWCR|=(1<<TWEN);
}


uint8_t i2c_inicia_com()
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTA);
	while (!(TWCR & (1<<TWINT)));
	
	if((TWSR & 0xF8) != TW_START)
	{
		return 0;
	}
	
	return 1;
}

void i2c_detener()
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	
}

uint8_t i2c_envia_dato(unsigned char dato)
{
	TWDR=dato;
	TWCR=(1<<TWEN)|(1<<TWINT);
	
	while (!(TWCR & (1<<TWINT)));
	
  	if( (TWSR & 0xF8) != TW_MR_SLA_ACK )
  	{
  		return  0;		//falla en recepción de ACK
  	}
  	  	
  	return 1;
}

unsigned char i2c_read_ack ()
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

unsigned char i2c_read_nack ()
{
	TWCR=(1<<TWEN)|(1<<TWINT);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

#endif /* I2C_H_ */


/*
 * I2C.h
 *
 * Created: 30/11/2018 13:11:03
 *  Author: F1
 */ 


#ifndef I2C_H_
#define I2C_H_

int dt,r_dato,var_dato=0;
uint8_t i2c_inicia_com();
void i2c_detener();
uint8_t i2c_envia_dato(unsigned char dato);
uint8_t i2c_read_ack ();
uint8_t i2c_read_nack ();


uint8_t i2c_inicia_com()
{
		//sda1
		DDRC&=~(1<<PC4);		//=0
		PORTC|=(1<<PC4);		//=1

		//scl1
		DDRC&=~(1<<PC5);		//=0
		PORTC|=(1<<PC5);		//=1

		_delay_us(0.5);

		//sda0
		DDRC|=(1<<PC4);	//=1
		PORTC&=~(1<<PC4);	//=0

		//_delay_us(0.5);
		return 1;
}

void i2c_detener()
{
		//sda0
		DDRC|=(1<<PC4);	//=1
		PORTC&=~(1<<PC4);	//=0
		//scl1
		DDRC&=~(1<<PC5);		//=0
		PORTC|=(1<<PC5);		//=1
		_delay_us(0.5);

		//sda1
		DDRC&=~(1<<PC4);		//=0
		PORTC|=(1<<PC4);		//=1

	//_delay_us(0.5);
}


uint8_t i2c_envia_dato(unsigned char dato)
{
	int b_wdt_int,var_dato,dt_aux=0;
	

		for (dt=0 ; dt<8 ; dt ++)
		{
			//scl0
			DDRC|=(1<<PC5);				//=1
			PORTC&=~(1<<PC5);			//=0
			//_delay_us(0.5);

			dt_aux=dato;
			dt_aux=dt_aux&0x80;
			if (dt_aux==0x80)
			{
				//sda1
				DDRC&=~(1<<PC4);		//=0
				PORTC|=(1<<PC4);		//=1
			}
			else
			{
				//sda0
				DDRC|=(1<<PC4);			//=1
				PORTC&=~(1<<PC4);		//=0
			}
			//_delay_us(0.5);
			//scl1
			DDRC&=~(1<<PC5);			//=0
			PORTC|=(1<<PC5);			//=1
			//_delay_us(0.2);

			dato <<= 1;
		}
		//scl0
		DDRC|=(1<<PC5);					//=1
		PORTC&=~(1<<PC5);				//=0
		//sda1
		DDRC&=~(1<<PC4);				//=0
		PORTC|=(1<<PC4);				//=1
		//_delay_us(1);

		//scl1
		DDRC&=~(1<<PC5);				//=0
		PORTC|=(1<<PC5);				//=1


		DDRC&=~(1<<PC4);
		var_dato=PIND;//&0X02;
		var_dato=var_dato&0X02;
		if (var_dato==2)
		{
			b_wdt_int=0;
		}
		else
		{
			b_wdt_int=1;
		}
		_delay_us(0.1);
		//scl0
		DDRC|=(1<<PC5);				//=1
		PORTC&=~(1<<PC5);			//=0
		//_delay_us(1);
		return b_wdt_int;
		//return 1;
	
}

unsigned char i2c_read_ack ()
{
	var_dato=0;
	r_dato=0;
	DDRC&=~(1<<PC4);
		for (dt=0 ; dt<8 ; dt ++)
		{
			//scl0
			DDRC|=(1<<PC5);	//=1
			PORTC&=~(1<<PC5);	//=0
		//	_delay_us(0.1);
			//scl1
			DDRC&=~(1<<PC5);		//=0
			PORTC|=(1<<PC5);		//=1
		//	_delay_us(0.1);

			var_dato=PINC;//&0X02;
			var_dato=var_dato&0X10;
			if (var_dato==0x10)
			{
				r_dato=r_dato+1;
			}
			//scl0
			DDRC|=(1<<PC5);	//=1
			PORTC&=~(1<<PC5);	//=0
			//_delay_us(1);
			if (dt<7)
			{
				r_dato=r_dato << 1;
			}
			
		}
		

		//sda0
		DDRC|=(1<<PC4);	//=1
		PORTC&=~(1<<PC4);	//=0

		//scl0
		DDRC|=(1<<PC5);	//=1
		PORTC&=~(1<<PC5);	//=0

		_delay_us(0.1);

		//scl1
		DDRC&=~(1<<PC5);		//=0
		PORTC|=(1<<PC5);		//=1

		_delay_us(0.1);

		//scl0
		DDRC|=(1<<PC5);	//=1
		PORTC&=~(1<<PC5);	//=0

		//_delay_us(1);
		//r_dato=1;
		return r_dato; 
		//Bandera_WDT=1;
}

unsigned char i2c_read_nack ()
{
	var_dato=0;
	r_dato=0;
		DDRC&=~(1<<PC4);
		for (dt=0 ; dt<8 ; dt ++)
		{
			//scl0
			DDRC|=(1<<PC5);	//=1
			PORTC&=~(1<<PC5);	//=0
		//	_delay_us(0.2);
			//scl1
			DDRC&=~(1<<PC5);		//=0
			PORTC|=(1<<PC5);		//=1
		//	_delay_us(0.1);

			var_dato=PINC;//&0X02;
			var_dato=var_dato&0X10;
			if (var_dato==0x10)
			{
				r_dato=r_dato+1;
			}
			//scl0
			DDRC|=(1<<PC5);	//=1
			PORTC&=~(1<<PC5);	//=0
			//_delay_us(1);
			if (dt<7)
			{
				r_dato=r_dato << 1;
			}
		}
		
		//sda1
		DDRC&=~(1<<PC4);		//=0
		PORTC|=(1<<PC4);		//=1

		//scl0
		DDRC|=(1<<PC5);	//=1
		PORTC&=~(1<<PC5);	//=0

		_delay_us(0.1);

		//scl1
		DDRC&=~(1<<PC5);		//=0
		PORTC|=(1<<PC5);		//=1
		_delay_us(0.1);

		//scl0
		DDRC|=(1<<PC5);	//=1
		PORTC&=~(1<<PC5);	//=0

		//_delay_us(1);
		//r_dato=0;
		return r_dato;
		//return 0;
}

#endif /* I2C_H_ */


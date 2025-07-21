/*
 * temperatura2.cpp
 *
 * Created: 01/12/2022 08:34:17 a. m. final
 * Author : i5-9400
 */ 

#define F_CPU 14745600UL
#define FOSC 14745600
#define BAUD 115200 // 9600 //
#define MYUBRR FOSC/16/BAUD-1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/wdt.h>

char	comando='0';
char	E_cancela[10]	= {"E1"};
uint8_t	flag_nb=0,v_timer_cp_H=0, v_timer_cp_L=0 , flag_triger=0, flag_dato=0, flag_modo=0,cont_pulsos=0, canal=2, aux_mod=0, flag_ok=0, aux_cont_ret=10, aux_cont_ret1=3;
uint16_t	v_timer_cp_0=0, v_timer_cp_1=0,v_timer_cp_T=0, HI_PWM=0, TRM_PWM=0;
int16_t aux_ret=0, aux_ret_0=0, aux_ret_1=0;
float	send_v_timer=0,temp=0,_tep0=0,_tep1=0,offset_res=0, res_term=0, temp_ant=0,offset_res_sis=-12;//offset_res_sis = -5
char	valor_C[10]={0},offset_res_C[10]={0};
static float Hi_Res=11474,Rf_res=12100;
static float Ra=0.001483032, Rb=0.000234995, Rc=0.0000001228041052;
	
#include "USART0.h"

uint16_t calc_PWM (uint8_t canal);

float Cal_temp ();
float Med_temp ();



ISR (USART_RX_vect)
{
	comando=UDR0;
	if (comando=='I')
	{
		if (flag_ok==0)
		{
			if (flag_nb==2)
			{
				send_string("calibrando...");
				send_char('\n');
			}
			else if (flag_nb==3)
			{
				send_string("midiendo...");
				send_char('\n');
			}
			else if (flag_nb==5)
			{
				//send_string("wtd");
				send_string("error sonda");
				send_char('\n');
			}
			else
			{
				send_string("error sonda");
				send_char('\n');
			}
			
		}
		else
		{
			flag_modo=1;
			dtostrf(temp,4,3,valor_C);
			send_string(valor_C);
			send_string("°C");
			send_char('\n');
			
		}
		
	} 
	else if (comando=='D')
	{
		flag_modo=0;
	}
	else if (comando=='P')
	{
		send_string("P");
		send_char('\n');
	}
	else if (comando=='C')
	{
		if (flag_ok==0)
		{
			send_string("error sonda");
			send_char('\n');
		}
		else
		{
			flag_modo=2;
		}
		
		/************************************************************************/
		/*  faltaria ver si es posible realizar un llamado de funcion para 
		poder calibrar le medicion de resistencia con base a los datos conocidos
		                                                                     */
		/************************************************************************/
	}
	else if (comando=='F')
	{
		send_string("F:Temp,2.0");
		send_char('\n');
	}
}

ISR (TIMER1_CAPT_vect)
{
	//PORTD^=(1<<PORTD4);
	//send_string("interrupcion");
	cli();
	v_timer_cp_L=ICR1L;
	v_timer_cp_H=ICR1H;
	if (flag_triger==0)
	{
		v_timer_cp_0=((v_timer_cp_H<<8)|(v_timer_cp_L));
		TCCR1B|= (1<<ICES1);
		flag_triger=1;
		//send_string(": 1");
	} 
	else if (flag_triger==1)
	{
		v_timer_cp_1=((v_timer_cp_H<<8)|(v_timer_cp_L));
		v_timer_cp_T=v_timer_cp_1-v_timer_cp_0;
		TCCR1B&=~(1<<ICES1);
		flag_triger=2;
		PORTC|=(1<<PORTC4);
		//send_string(": 2");
	}
	//send_char('\n');
	sei();
}

ISR (WDT_vect)
{
	wdt_reset();
	flag_ok=0;
	flag_nb=5;
	//send_string("Reset");// se comento porque genera error al conectar todas las interfaces 
	//send_char('\n');
}


int main(void)
{
	cli();
	//usart_init();
	/*************************configuracion de pines ***********************************/
	//_delay_ms(10);
	USART_Init(MYUBRR);
	UCSR0B =  (1<<RXEN0) | (1<<TXEN0)|(1<<RXCIE0);
	TCCR1B =	(1<<ICNC1)  | (1<<CS11);	// PRESCALER A 8 CON FLANCO DESCENDENTE CON CANCELACION DE RUIDO && (0<<ICES1)
	TIMSK1 =	(1<<ICIE1);
	DDRB|= (1<<DDB2) |(1<<DDB1);
	PORTB&=~(1<<PORTB1);
	PORTB&=~(1<<PORTB2);
	DDRC|= (1<<DDC2) | (1<<DDC4);//DDC0
	PORTC&=~(1<<PORTC2) | (1<<PORTC4); //PORTC2);//PORTC0
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out
	*/
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x55;
	DDRD|= (1<<DDD6) | (1<<DDD7) | (1<<DDD3) | (1<<DDD4);
	PORTD|=(1<<PORTD7);
	PORTD&=~(1<<PORTD3);
	PORTD&=~(1<<PORTD4);
	TCCR0A|= (1<<WGM01) | (1<<WGM00) | (1<<COM0A1) | (1<<COM0A0);
	TCCR0B|= (1<<CS02); // | (1<<CS00);
	//flag_nb=1;
	sei();
	/********************fin de configuracion de pines ***********************************/

	send_string("INICIANDOO");
	send_char('\n');
	Cal_temp();
	Med_temp();
	temp_ant=temp;
	while(1)
	{
		//TODO:: Please write your application code
		/************************************************************************/
		Med_temp();
		flag_nb=4;
		if (flag_ok>aux_cont_ret)
		{
			flag_ok=aux_cont_ret1;
			
		}
		flag_ok++;
		//wdt_reset();
		if (flag_modo==1)
		{
			flag_modo=0;
			aux_mod++;
			if (aux_mod>10)
			{
				//flag_modo=3;
				flag_modo=2;
				aux_mod=0;
			}
			//_delay_ms(2000);
		}
		else if (flag_modo==2)
		{
			Cal_temp();
			dtostrf(offset_res,4,3,valor_C);
			send_string(valor_C);
			send_string("valor offset");
			send_char('\n');
			flag_modo=0;
		}
		                                                                
		/************************************************************************/
		
	}
}


uint16_t calc_PWM (uint8_t canal)
{
	uint16_t PWM=0;
	wdt_reset();
	PORTC|=(1<<PORTC4);
	_delay_ms(10);
	/************************************************************************/
	/* falta activar todos los canales para descargar capasitor y activar el 555
	luego seleccionamos el canal correspondiente y leemos el ancho de pulso */
	/************************************************************************/
	flag_dato=0;
	if (canal==0)
	{
		PORTD&=~(1<<PORTD3);
		PORTD&=~(1<<PORTD4);
		PORTC&=~(1<<PORTC2);//PORTC0
		PORTB&=~(1<<PORTB1);
		PORTB|=(1<<PORTB2);
		PORTC&=~(1<<PORTC4);
	} 
	else if (canal==1)
	{
		PORTD|=(1<<PORTD3);
		PORTD&=~(1<<PORTD4);
		PORTC&=~(1<<PORTC2);
		PORTB&=~(1<<PORTB2);
		PORTB|=(1<<PORTB1);
		PORTC&=~(1<<PORTC4);
	}
	else if (canal==2)
	{
		PORTD|=(1<<PORTD4);
		PORTD&=~(1<<PORTD3);
		PORTB&=~(1<<PORTB2);
		PORTB&=~(1<<PORTB1);
		PORTC|=(1<<PORTC2);
		PORTC&=~(1<<PORTC4);
	} 
	else if (canal==3)
	{
		PORTD|=(1<<PORTD3);
		PORTD|=(1<<PORTD4);
		PORTB&=~(1<<PORTB2);
		PORTB&=~(1<<PORTB1);
		PORTC&=~(1<<PORTC2);
		PORTC&=~(1<<PORTC4); // en espera por recomendacion //
		flag_dato=1;
		PWM=0;
	}
	else if (canal==4)
	{
		PORTD|=(1<<PORTD3);
		PORTD|=(1<<PORTD4);
		PORTB|=(1<<PORTB2);
		PORTB|=(1<<PORTB1);
		PORTC|=(1<<PORTC2);
		PORTC&=~(1<<PORTC4); // en espera por recomendacion //
	}
	else
	{
		flag_dato=1;
	}
	flag_triger=0;
	while(flag_dato==0)
	{
		//send_string("espera de interrupcion");
		if (flag_triger==2)
		{
			_delay_us(1000);
			PWM=v_timer_cp_T;
			flag_dato=1;	
		}
	}
	return PWM;
}

float Cal_temp()
{
	int lim_cnt=50, cont_pulsos=0;
	_tep1=0;
	calc_PWM(3);
	_delay_ms(10);
		while(cont_pulsos<lim_cnt)
		{
			HI_PWM=	calc_PWM(0);
			_tep1=(float)HI_PWM+_tep1;
			cont_pulsos++;
			calc_PWM(4);
			calc_PWM(3);
				flag_nb=2;
		}
	_tep1=_tep1/(float)lim_cnt;

	cont_pulsos=0;
	_tep0=0;
	_delay_ms(10);
		while(cont_pulsos<lim_cnt)
		{
			HI_PWM=calc_PWM(1);
			_tep0=(float)HI_PWM+_tep0;
			cont_pulsos++;
			calc_PWM(4);
			calc_PWM(3);
		}
		
	_tep0=_tep0/(float)lim_cnt;
	res_term=_tep0/_tep1;
	res_term=res_term*Hi_Res;
	//res_term=_tep0;
	offset_res=Rf_res-res_term+offset_res_sis;
	_delay_ms(10);
	
}

float Med_temp()
{
	
int lim_cnt=50, cont_pulsos=0;
	_tep1=0;
	calc_PWM(3);
	_delay_ms(10);
	while(cont_pulsos<lim_cnt)
	{
		HI_PWM=	calc_PWM(0);
		_tep1=(float)HI_PWM+_tep1;
		cont_pulsos++;
		calc_PWM(4);
		calc_PWM(3);
		flag_nb=3;
	}
	_tep1=_tep1/(float)lim_cnt;
				
	cont_pulsos=0;
	_tep0=0;
	_delay_ms(10);
	while(cont_pulsos<lim_cnt)
	{
		HI_PWM=calc_PWM(2);
		_tep0=(float)HI_PWM+_tep0;
		cont_pulsos++;
		calc_PWM(4);
		calc_PWM(3);
	}
	_tep0=_tep0/(float)lim_cnt;
	res_term=_tep0/_tep1;
	res_term=res_term*Hi_Res;
	res_term=res_term+offset_res;
	temp=1/(Ra+Rb*(log(res_term))+Rc*(pow(log(res_term),3)));
	temp=temp-273.15;
}
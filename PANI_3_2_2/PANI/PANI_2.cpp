/*
 * PANI_2.cpp
 *
 * Created: 17/08/2023 02:30:14 p.m.
 * Author : Tonatiuh Velazquez
 *
 *
 * 07/11/2022 
 *		Se agrega respuesta a comando 'M'
 *		Se desactiva cierre de v?lvula al final de la medici?n
		debug
 *
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "USART0.h"
//#include "I2C.h"
#include "I2Cuser.h"
#include "Sensores.h"


#define NPA700_R    0x51
#define NPA700_W    0x50

char texto_info[10]		= {"I:PNI,"};
char version_pani[10]	= {"3.2.2"};		// correccion a desfogue

char	comando='0';

char	pres_str[10]={0};
char	m_pres_str[10]={0};
char	sist_str[10]={0};
char	valor_p[10]={0};
char	valor_m[10]={0};

char E_cancela[10]		= {"E0"};
char E_mang_des[10]		= {"E1"};
char E_F_Valv_aliv[10]	= {"E2"};
char E_F_Valv_prop[10]	= {"E3"};
char E_F_Valv_ayp[10]	= {"E4"};
char E_Bomba[10]		= {"E5"};
char E_Tmr_PNIA[10]		= {"E6"};
char E_Sobre_presA[10]	= {"E7"};
char E_Sig_Deb[10]		= {"E8"};
char E_calc_pres[10]	= {"E9"};
char E_Tmr_PNIB[10]		= {"E10"};
char E_Sobre_presB[10]	= {"E11"};
char E_fuga[10]			= {"E12"};
char E_Conf[10]			= {"E13"};
	

int		pres_array[100]={0};//,pres_array1[230]={0};
float	delta_array[50]={0};//,m_array1[230]={0};
float	delta_pres_0=5;//,m_array1[230]={0};
float	delta_pres_1=5;
int		timer_array[100]={0};
int		limite_presion=190;
int		limite_presion2=120;
int		max_dif=0,aux_port=0;

int		cont_timer=0,  fin_bp=0;
float	m_sist=0;
float	pres_ant=0,max_pres=0,min_pres=0, pres_act=0, pres_min=0, dif_pres=0, aux_pres_max=0,dif_pres_max=0.30,pres_prom,value_dif_max=0;
int8_t	inicio_med=0,rx_pos=0,inicio_calc=0,flag_pulsos=0,flag_neo=0,flag_modo=0,aux_PORTD=0;
int8_t	flag_cancela=0,flag_escalon=0;
uint8_t	 aux_ret_0=0,LIM_pwm=0,aux_iarray=0,open_val=30,close_val=24;
int flag_start=0,flag_2ms=0;
uint32_t cont_timer1=0,cont_timer2=0,duracion_pulso=0,cont_escalon=0,tiempo_limtotal=45000,tiempo_limparc=100,pres_lim1=5,pres_lim2=10,pres_lim3=20,pres_lim4=30,aux_con_pulso=0;
int flag_DB_PUMP=-1,flag_error_tp=0;
int flag_DB_VAL=-1;
int debug_B =0, flag_confg=0;

ISR (USART_RX_vect)
{
	comando=UDR0;
	
	if (comando =='M')
	{
		inicio_med=1;
		flag_cancela=0;
		inicio_calc=0;
		flag_confg=1;
		flag_error_tp=0;
		send_char('M');
		send_char('\n');
		_delay_ms(500);		
	}
	if (comando =='D')
	{
		inicio_med=0;
		inicio_calc=0;
		//PORTB = 0x02;
		send_string(E_cancela);
		send_char('\n');
		flag_escalon=1;
	}

	if (comando =='V')			// cerrar v?lvula
	{
		PORTB &=~(0xFE);
		send_char('V');
		send_char('\n');
		flag_cancela=1;
	}

	if (comando =='A')
	{
		flag_confg=1;
		limite_presion=240;
		limite_presion2=limite_presion/2;
		send_char('A');
		send_char('\n');
		flag_neo=0;
		flag_modo=64;
		tiempo_limparc=300;
		pres_lim1=10;
		pres_lim2=30;
		pres_lim3=45;
		pres_lim4=70;
		_delay_ms(500);	
	}

	if (comando =='N')
	{
		flag_confg=1;
		limite_presion=160;
		limite_presion2=limite_presion/2;
		send_char('N');
		send_char('\n');
		flag_neo=0;
		flag_modo=16;
		tiempo_limparc=300;
		pres_lim1=10;
		pres_lim2=30;
		pres_lim3=45;
		pres_lim4=70;
		_delay_ms(500);	
	}

	if (comando =='B')
	{
		flag_confg=1;
		limite_presion=120;
		limite_presion2=limite_presion/2;
		flag_neo=1;
		flag_modo=8;
		send_char('B');
		send_char('\n');
		tiempo_limparc=100;
		pres_lim1=5;
		pres_lim2=10;
		pres_lim3=20;
		pres_lim4=30;
		_delay_ms(500);	
	}

	if (comando =='I')
	{
		send_string(texto_info);
		send_string(version_pani);
		send_char('\n');
	}
	if (comando =='P')
	{
		send_char('P');
		send_char('\n');
	}	
	if (comando =='G')
	{
		send_char('G');
		send_char('\n');
		inicio_med=5;
	}
	if (comando =='X')
	{
		send_char('X');
		send_char('\n');
		inicio_med=6;
		PORTB = 0x03;//0x03;	//activa motor 1 y v?lvula
		tiempo_limtotal=0;//90000
		aux_ret_0=255;
		//OCR0B=255;
	}
	if (comando =='+')
	{
		send_char('+');
		send_char('\n');
		aux_ret_0+=1;
		//OCR0B=255;
	}
	if (comando =='-')
	{
		send_char('-');
		send_char('\n');
		aux_ret_0-=1;
		//OCR0B=255;
	}
///////////////////////////////////////////debug
	
}

 ISR (TIMER2_COMPA_vect)
 {
	 if (flag_start!=0)
	 {
		 flag_2ms=1;
		if (cont_escalon>1000)
		{
			cont_escalon=0;
			flag_escalon=1;
		}
		cont_escalon++;
	 }
	 
	 if (inicio_med==0)
	 {
		 cont_timer1=0;
	 }
	 else
	 {
		 cont_timer1++;
	 }
	 if (cont_timer2<2000)
	 {
		 cont_timer2++;
	 }
	 
	 				
 }
float DerivadaFuncion (float derivacion);

int main(void)
{
	float	pres_npa=0;
	float	p_med=0;
	float	p_sist=0;
	float	limi_dias=0.6;//0.65
	float	limi_sist=0.7;//0.6
	int		send_count=0; // contador para enviar dato de presion mientras se mide

	int p_dias=0,flag_no_pulso=0;
	int flag_sist=0;
	int flag_pulso_on=0;
	float  aux_dif_max=0,index_max=0;

	cont_escalon=0;
	flag_escalon=0;
	flag_pulsos=0;
	flag_neo=0;
	flag_modo=0;
	aux_PORTD=5;

	int	i_array=0;

	
	DDRB = 0b00000111;		//PB0  v?lvula   PB1 motor

	DDRD	= 0b00100000;
	PORTD	= 0x00;
	TCCR0A	= 0x23;
	TCCR0B	= 0x01;
	TCCR2A	= 0x02;
	OCR2A	= 230;
	TCCR2B	= 0x05;
	TIMSK2	= 0x02;
	OCR0B=0x01;
	PORTB = 0x03;//0x02;
	cli();
		
	usart_init();

	UCSR0B =  (1<<RXEN0) | (1<<TXEN0)|(1<<RXCIE0);
	
	sei();
    
	//i2c_inicio();	

	read_NPA700est(NPA700_R);

	_delay_ms(100);

		send_string(texto_info);
		send_string(version_pani);
		send_char('\n');
		limite_presion=190;

	while (1) 
    {
		pres_npa=read_NPA700(NPA700_R);
		pres_act=pres_npa;
		if ((pres_act>315)||(pres_act>limite_presion))
		{
			inicio_med=0;
			flag_DB_PUMP=-1;
			debug_B=PORTB;
			debug_B = debug_B&(0xFA);
			PORTB=debug_B;
			send_string(E_Sobre_presB);		//E11 sobrepresion
			send_char('\n');
			send_char('\r');
		}
		///////////////rev de fallas uC de seg///////////////////////////
		//PIND
		aux_PORTD=PIND;
		aux_PORTD=aux_PORTD&(0b11011100);
		if (flag_confg==1)
		{
			flag_confg=0;
			if (flag_modo!=aux_PORTD)
			{
				if (aux_PORTD=0x80)
				{
					send_string(E_Sobre_presB); //
					send_char('\n');
				}
				else if (aux_PORTD=0x04)
				{
					send_string(E_Tmr_PNIB); //
					send_char('\n');
				}
				else
				{
					send_string(E_Conf); //
					send_char('\n');
				}
				inicio_med=4;
				
			}
			
		}
		if ((inicio_med!=0)&&inicio_med<5)
		{
			send_count++;
			if (send_count>100)
			{
				send_count=0;
				send_string("R: ");
				dtostrf(pres_npa,3,0,valor_m);
				send_string(valor_m);
				send_string(", ");
				dtostrf(aux_ret_0,3,0,valor_m);
				send_string(valor_m);
				send_char('\n');
				send_char('\r');
			}
			
			
			if (cont_timer1>tiempo_limtotal)
			{
				send_string(E_fuga);		//E12 sobrepresion
				send_char('\n');
				inicio_med=0;
			}
		}
		if (inicio_med==0)
		{
			limite_presion2=limite_presion/2;
			PORTB=0;
			OCR0B=1;
		}
		else if (inicio_med==1)
		{
			if (flag_neo==1)
			{
				PORTB = 0x06;//0x06;	//activa motor 2 y v?lvula
				tiempo_limtotal=45000;
				dif_pres_max=0.145;
			}
			else
			{
				PORTB = 0x03;//0x03;	//activa motor 1 y v?lvula
				tiempo_limtotal=90000;
				dif_pres_max=0.30;
			}
			OCR0B=255;
			
			cont_timer2=0;
			flag_sist=0;
			flag_no_pulso=0;
			flag_pulso_on=0;
			index_max=0;
			if ((pres_act>3)&&(flag_error_tp==0))
			{
				flag_error_tp=1;
			}
			if (pres_act>5)
			{
				flag_error_tp=2;
			}
			if ((cont_timer1>500)&&(pres_act<0.3))
			{
				if (flag_error_tp==0)
				{
					send_string(E_Bomba);		//E4 motor no enciende
					send_char('\n');
					inicio_med=0;
				}
				else if (flag_error_tp==1)
				{
					send_string(E_F_Valv_aliv);		//E4 motor no enciende
					send_char('\n');
					inicio_med=0;
				}
				else if (flag_error_tp==2)
				{
					send_string(E_F_Valv_prop);		//E4 motor no enciende
					send_char('\n');
					send_string(E_mang_des);		//E12 fuga
					send_char('\n');
					inicio_med=0;
				}
			}
			if (pres_act>limite_presion2)
			{
				debug_B=PORTB;
				debug_B = debug_B&(0xFA);
				PORTB=debug_B;
				inicio_med=2;
			}
		}
		else if (inicio_med==2)
		{
			dif_pres=(DerivadaFuncion(pres_npa));
			
			if ((cont_timer2>750)&&(dif_pres>dif_pres_max))//0.05
			{
				if (flag_neo==1)
				{
					limite_presion2=limite_presion2+10;
					inicio_med=1;
				} 
				else
				{
					limite_presion2=limite_presion2+20;
					inicio_med=1;
				}
				
			}
			if (cont_timer2==2000)
			{
				inicio_med=3;
				cont_timer2=0;
				flag_escalon=0;
				aux_pres_max=pres_act;
				i_array=0;
				index_max=0;
				aux_dif_max=0;
			}	
		}
		else if (inicio_med==3)
		{
			dif_pres=(DerivadaFuncion(pres_npa));
			
			if (pres_act<35)
			{
				inicio_med=4;
				flag_DB_PUMP=-1;
				debug_B=PORTB;
				debug_B = debug_B&(0xFA);
				PORTB=debug_B;
			}
	
			while(flag_escalon==0)
			{
				flag_pulso_on=0;
				pres_array[i_array]=0;
				delta_array[i_array]=0;
				aux_con_pulso=0;
				cont_timer2=0;
				pres_npa=read_NPA700(NPA700_R);
				pres_act=pres_npa;
				if (pres_act<aux_pres_max)
				{
					flag_escalon=1;
				}
				aux_ret_0=OCR0B;

				if (aux_ret_0>10)
				{
					if (aux_ret_0>250)
					{
						//aux_ret_0=30;//funcion de open orig 215
						aux_ret_0=open_val;
						OCR0B=aux_ret_0;
					}
					if (aux_ret_0>close_val)
					{
						aux_ret_0=aux_ret_0-1;
					}
					
					OCR0B=aux_ret_0;
				}
				else
				{
					OCR0B=5;
				}
			}
			while(flag_escalon==1)
			{
				pres_array[i_array]=0;
				delta_array[i_array]=0;
				aux_con_pulso=0;
				cont_timer2=0;
				pres_npa=read_NPA700(NPA700_R);
				pres_act=pres_npa;
				aux_ret_0=OCR0B;
				if (aux_ret_0<254)
				{
					aux_ret_0=aux_ret_0+1;
					OCR0B=aux_ret_0;
				}
				else
				{
					OCR0B=255;
					flag_escalon=2;
					flag_pulsos=0;
				}
			}
			if ((cont_timer2>180)&&(cont_timer2<190))
			{
				min_pres=pres_act;
				max_pres=pres_act;
			}
			if ((dif_pres>dif_pres_max)&&(cont_timer2>200))
			{
				if (flag_pulsos==0)
				{
					flag_pulsos=1;
				} 
				else if (flag_pulsos==2)
				{
					
					if ((i_array>(index_max+2))&&(i_array>3))
					{////////////////////funcion de corte se debe editar
						
						if ((delta_array[i_array]<(aux_dif_max*limi_dias))||(delta_array[i_array]<0.1))
						{
							if (i_array>4)
							{
								inicio_med=4;
								flag_DB_PUMP=-1;
								debug_B=PORTB;
								debug_B = debug_B&(0xFA);
								PORTB=debug_B;
							}
						}
						else
						{
							flag_no_pulso=0;
						}
					}
					if (delta_array[i_array]>aux_dif_max)
					{
						aux_dif_max=delta_array[i_array];
						index_max=i_array;
					}
					if (i_array==0)
					{
						pres_array[i_array]=int(pres_npa);
						delta_array[i_array]=max_pres-min_pres;
					}
					cont_timer2=2000;
					flag_pulsos=3;
					i_array++;
					flag_no_pulso=0;
					//aux_pres_max=pres_act-5;
					flag_pulso_on=1;
					if (pres_act>100)
					{
						//aux_pres_max=pres_act-5;
						aux_pres_max=pres_act-delta_pres_0;
					}
					else
					{
						//aux_pres_max=pres_act-10;
						aux_pres_max=pres_act-delta_pres_1;
					}
				}
			}
			if (flag_pulsos==1)
			{
				aux_con_pulso++;
				if (pres_act>max_pres)
				{
					max_pres=pres_act;
					pres_array[i_array]=int(pres_npa);
					delta_array[i_array]=max_pres-min_pres;
				}
				if (aux_con_pulso>400)
				{
					aux_con_pulso=0;
					flag_pulsos=2;
				}
			}
			if (cont_timer2>1500)
			{
				flag_escalon=0;
				cont_timer2=0;
				if (flag_pulso_on==0)
				{
					if (pres_act>100)
					{
						aux_pres_max=pres_act-10;
					}
					else
					{
						aux_pres_max=pres_act-10;
					}
					//aux_pres_max=pres_act-5;
				}
				max_pres=0;
				flag_no_pulso++;
				if ((flag_no_pulso>3)&& (i_array>5))
				{
					inicio_med=4;
					flag_DB_PUMP=-1;
					debug_B=PORTB;
					debug_B = debug_B&(0xFA);
					PORTB=debug_B;
				}
			}	
		}
		else if (inicio_med==4)
		{
			send_string("inicio de calculos: ");
			send_char('\n');
			send_string("delta max: ");
			dtostrf(aux_dif_max,2,3,valor_m);
			send_string(valor_m);
			send_char('\n');
			if (i_array<5)
			{
				send_string(E_calc_pres);		//E10 calculo erroneo
				send_char('\n');
			}
			for (int i=0;i<i_array;i++)
			{
				send_string("pulso numero: ");
				dtostrf(i,2,0,valor_m);
				send_string(valor_m);
				send_string(" Pres: ");
				send_string(",");
				dtostrf(pres_array[i],3,0,valor_m);
				send_string(valor_m);
				//send_string(" Dif: ");
				send_string(",");
				dtostrf(delta_array[i],0,5,valor_m);
				send_string(valor_m);
				send_string(",");
				//send_string("pulso numero: ");
				dtostrf(i,2,0,valor_m);
				send_string(valor_m);
				send_char('\n');
				send_char('\r');
				if (flag_neo==1)
				{
					p_sist=pres_array[0];
					p_dias=pres_array[i_array-1];
				} 
				else
				{
					if ((flag_sist==0)&&(delta_array[i]>(aux_dif_max*limi_sist)))
					{
						p_sist=pres_array[i];
						flag_sist=1;
						
					}
					//p_dias=pres_array[i_array-1];
					p_dias=pres_array[i_array-2];
				}
				
			}
			inicio_med=0;
			p_med=p_dias+(p_sist-p_dias)/3;
			send_string("m:");
			dtostrf(p_sist,2,0,sist_str);
			send_string(sist_str);
			send_char(',');
			dtostrf(p_dias,2,0,valor_m);
			send_string(valor_m);
			send_char(',');
			dtostrf(p_med,2,0,valor_m);
			send_string(valor_m);
			send_char('\n');
			send_char('\r');
		}
		else if (inicio_med==5)
		{
			///////////////////////////////////////
			PORTB = 0x03;//0x03;	//activa motor 1 y v?lvula
			tiempo_limtotal=0;//90000
			OCR0B=255;
			while(pres_npa<120)
			{
				pres_npa=read_NPA700(NPA700_R);
				send_string("I: ");
				dtostrf(pres_npa,3,0,valor_m);
				send_string(valor_m);
				send_char('\n');
				send_char('\r');
			}
			PORTB = 0x02;//0x03;	//activa  valvula
			while(tiempo_limtotal<10000)
			{
				tiempo_limtotal++;
				pres_npa=read_NPA700(NPA700_R);
				send_string("Z: ");
				dtostrf(pres_npa,3,0,valor_m);
				send_string(valor_m);
				send_string(", C: ");
				dtostrf(tiempo_limtotal,3,0,valor_m);
				send_string(valor_m);
				send_char('\n');
				send_char('\r');
			}
			inicio_med=4;
			//////////////////////////////////////
			
		}
		else if (inicio_med==6)
		{
			///////////////////////////////////////
			//PORTB = 0x03;//0x03;	//activa motor 1 y v?lvula
			//tiempo_limtotal=0;//90000
			//OCR0B=255;
			OCR0B=aux_ret_0;
			pres_npa=read_NPA700(NPA700_R);
			send_string("P: ");
			dtostrf(pres_npa,3,0,valor_m);
			send_string(valor_m);
			send_string(", V: ");
			dtostrf(aux_ret_0,3,0,valor_m);
			send_string(valor_m);
			send_char('\n');
			send_char('\r');
			if (pres_npa>120)
			{
				send_string(", V: ");
				PORTB = 0x02;//apagar motor
			}
			//////////////////////////////////////
			
		}
	}		
}

float DerivadaFuncion (float derivacion)
{
	int i;
	float y;
	static float fx_derv[100];

	y = ((derivacion*2) + fx_derv[98] - fx_derv[1] - (fx_derv[0]*2))/8;
	for (i = 0; i < 99; i++)
	{
		fx_derv[i] = fx_derv[i + 1];
	}
	fx_derv[99] = derivacion;

	return(y);
}

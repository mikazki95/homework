/*
 * PANI.cpp
 *
 * Created: 24/10/2022 02:30:14 p.m.
 * Author : Gaby Morales
 *
 *
 * 07/11/2022 
 *		Se agrega respuesta a comando 'M'
 *		Se desactiva cierre de vúlvula al final de la medición
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
char version_pani[10]	= {"2.3.0"};		// corrección respuesta al comando M

char	comando='0';

char	pres_str[10]={0};
char	m_pres_str[10]={0};
char	sist_str[10]={0};
char	valor_p[10]={0};
char	valor_m[10]={0};

char E_cancela[10]	= {"E1"};
char E_pr_neg[10]	= {"E2"};
char E_pr_exce[10]	= {"E3"};
char E_fuga[10]		= {"E4"};
char E_lib_lenta[10] = {"E5"};
char E_lib_rapida[10] = {"E6"};
char E_calc_pr[10]	= {"E7"};
char E_calc_FC[10]	= {"E8"};

int		pres_array[100]={0};//,pres_array1[230]={0};
int		delta_array[100]={0};//,m_array1[230]={0};
int		timer_array[100]={0};
int		limite_presion=190;
int		max_dif=0;

int		cont_timer=0,cont_timer1=0, duracion_pulso=0, fin_bp=0;
float	m_sist=0;
float	pres_ant=0,max_pres=0,min_pres=0, pres_act=0, pres_min=0, dif_pres=0;
int8_t	inicio_med=0,rx_pos=0,inicio_calc=0;
int8_t	flag_cancela=0;

ISR (USART_RX_vect)
{
	comando=UDR0;
	
	if (comando =='M')
	{
		inicio_med=1;
		flag_cancela=0;
		send_char('M');
		send_char('\n');
		TCCR0B = 0X05;
		PORTB = 0b00000010;	//activa motor y válvula
	}
	if (comando =='D')
	{
		inicio_med=3;
		inicio_calc=0;
		send_string(E_cancela);
		send_char('\n');
	}

	if (comando =='V')			// cerrar válvula
	{
		PORTB &=~(0xFE);
		send_char('V');
		send_char('\n');
		flag_cancela=1;
	}

	if (comando =='A')
	{
		limite_presion=190;
		send_char('A');
		send_char('\n');
	}

	if (comando =='N')
	{
		limite_presion=160;
		send_char('N');
		send_char('\n');
	}

	if (comando =='B')
	{
		limite_presion=120;
		send_char('B');
		send_char('\n');
	}

	if (comando =='I')
	{
		send_string(texto_info);
		send_string(version_pani);
		send_char('\n');
	}
	
	if (comando=='P')
	{
		send_char('P');
		send_char('\n');
	}
}

ISR (TIMER0_OVF_vect)
{
	cont_timer++;
	cont_timer1++;
	TCNT0 = 184;
}


int main(void)
{
float	p_med=0;
float	suma_BPM=0;
float	pres_npa=0;
//float	m_pres=0;
float	p_sist=0;
float	limi_dias=0, BPM=0;
float	limi_sist=0;

int indice_max=0;
int p_dias=0;
int flag_sist=0;
int max_delta=0;
int aux_dif=0;
//int p_act_int=0;
//int flag_max=0;

int flag_start=0;
int	i_array=0, indice=0;

	DDRB = 0b00000011;		//PB0  válvula   PB1 motor

	DDRD = 0b01000000;
	PORTD = 0x40;
	
	TCNT0 = 184;
	TCCR0A = 0x00;
	TCCR0B = 0x00;
	TIMSK0 = 0x01;

	cli();
		
	usart_init();

	UCSR0B =  (1<<RXEN0) | (1<<TXEN0)|(1<<RXCIE0);
	
	sei();
    
	//i2c_inicio();	

	read_NPA700est(NPA700_R);

	_delay_ms(100);

// 	send_string("E0");
// 	send_char('\n');

	while (1) 
    {
		if (inicio_med==1)
 		{
	 		pres_npa=read_NPA700(NPA700_R);
			pres_act=pres_npa;
 			/*dtostrf(pres_act,2,0,valor_p);
 			send_string("R:");
 			send_string(valor_p);
 			send_char('\n');*/
			
			/*dtostrf(pres_act,2,0,valor_p);
			send_string(valor_p);
			send_char(',');
			dtostrf(duracion_pulso,2,0,valor_p);
			send_string(valor_p);
			send_char(',');
			dtostrf(i_array,2,0,valor_p);
			send_string(valor_p);
			send_char('\r');
			*/
			if (inicio_calc==0)
			{
				cont_timer1=0;
				if (pres_act>=limite_presion)
				{
					inicio_calc = 1;
					PORTB = 0b00000000;	//desactiva motor y válvula

					if (pres_act>limite_presion+10)
					{
						inicio_med=3;
						send_string(E_pr_exce);		//E3
						send_char('\n');
					}
				}
				else if (pres_act<-2)
				{
					inicio_med=3;
					inicio_calc=0;
					send_string(E_pr_neg);			//E2
					send_char('\n');
				}
				else if ((pres_act<20)&&(pres_act>-2))
				{
					if (cont_timer>1600)
					{
						inicio_med=3;
						inicio_calc=0;
						send_string(E_fuga);		//E4
						send_char('\n');
					}
				}
				else if (pres_act<0.9*limite_presion)		// se agrega condición para detectar otro tipo de fuga
				{
					if ((cont_timer>2800)&&(inicio_calc==0))
					{
						inicio_med=3;
						inicio_calc=0;
						send_string(E_fuga);		//E4
						send_char('\n');
					}
				}
			}

						
			if (inicio_calc==1)
			{
				if (flag_start==0)
				{
					_delay_ms(500);
					flag_start=1;
					pres_min=pres_act;
					dif_pres=0;
					pres_ant=pres_act;
					cont_timer=0;
					cont_timer1=0;
					max_delta=0;
				} 
				else 
				{
					if (cont_timer1>610)
					{
						dif_pres=pres_act-pres_ant;
						pres_ant=pres_act;

						if (dif_pres>-4)		// -6  CAMBIO PARA MSV de Ing. Hernández 24/11/22
						{
							inicio_med=3;
							inicio_calc=0;
							send_string(E_lib_lenta);		//E5
							send_char('\n');
						}
						else if (dif_pres<-65)//65
						{
							inicio_med=3;
							inicio_calc=0;
							send_string(E_lib_rapida);		//E6
							send_char('\n');
						}
						cont_timer1=0;
					}
					if ((pres_act+0.2)<pres_min)
					{
						pres_min=pres_act;
						
						//if (duracion_pulso>=3)
						//{
							if (flag_start==2)
							{							
								if (delta_array[i_array]*100<max_dif*45)//55
								{
									fin_bp++;
									if (fin_bp>3)
									{
										inicio_med=2;
										inicio_calc=0;
									}
										
								}
								timer_array[i_array]=cont_timer;
								i_array++;
								flag_start=1;
								cont_timer=0 ;
							}
						//flag_start=1;
						//duracion_pulso=0;
						//}
						//flag_start=1;
						duracion_pulso=0;
					}
					else 
					{	
						duracion_pulso++;
						if (duracion_pulso>=3)
							{
							aux_dif=(int)100*(pres_act-pres_min);
							if (delta_array[i_array]<aux_dif)
							{
								if (max_dif<delta_array[i_array])
								{
									max_dif=delta_array[i_array];
								}
								flag_start=2;
								indice=i_array;
								delta_array[i_array]=aux_dif;
								pres_array[i_array]=(int)pres_act;
							}
						}
					}
				}
			}

			_delay_ms(25);  //valor ininial de pruebas
 		}
		else if (inicio_med==2)
		{

			flag_sist=0;	
			flag_start=0;
			cont_timer1=0;
			cont_timer=0;
			TCCR0B = 0x00;
			indice=i_array;
			for (int i=0;i<i_array;i++)
			{
				dtostrf(pres_array[i],2,0,valor_m);
				send_string(valor_m);
				send_string(",");
				dtostrf(delta_array[i],2,0,valor_m);
				send_string(valor_m);
				send_char('\r');
				if (i>0)
				{					
					suma_BPM+=12168/(float)timer_array[i];
				}
				if (delta_array[i]>max_delta)
				{
					max_delta=delta_array[i];
					indice_max=i;
				}

			}

			limi_sist=(float)delta_array[indice_max]*0.495;//0.495
			limi_dias=(float)delta_array[indice_max]*0.68;//0.68
			send_string("iniciando calculo sis");
			send_char('\r');
			
			for (int j=0;j<=i_array;j++)
			{
					dtostrf(pres_array[j],2,0,valor_m);
					send_string(valor_m);
					send_string(",");
					dtostrf(delta_array[j],2,0,valor_m);
					send_string(valor_m);
					send_char('\r');
				if ((flag_sist==0)&&(delta_array[j]>limi_sist))
				{
					send_string("sis");
					send_char('\r');
					flag_sist=1;
					if (j<1)
					{
						p_sist=pres_array[j];
					} 
					else
					{
						
						p_sist=pres_array[j-1];
					}
					j=i_array;
				}/*
					if (j==i_array)
					{
						send_string("dias");
						send_char('\r');
						p_dias=pres_array[j];
					}
					else if ((j>indice_max)&&(delta_array[j]<limi_dias))
					{
						p_dias=pres_array[j];		//[j-1];
						j=i_array+1;
					}*/
			}
			send_string("iniciando calculo dias");
			send_char('\r');
			for (int j=i_array;j>1;j--)
			{
				if (j==indice_max)
				{
					send_string("error de dias");
					send_char('\r');
					p_dias=pres_array[j];
					j=0;
				}
				else if ((j>indice_max)&&(delta_array[j]>limi_dias))
				{
					send_string("dias ok");
					send_char('\r');
					p_dias=pres_array[j];		//[j-1];
					j=0;
				}
			}
			if (i_array<4)
			{
				inicio_med=3;
				inicio_calc=0;
  				send_string(E_calc_pr);		//E7
  				send_char('\n');
				send_string(E_calc_FC);		//E8
				send_char('\n');
 				
			}
			else if ((p_sist==0)||(p_dias==0))
			{
				inicio_med=3;
				inicio_calc=0;
				send_string("cero");
				send_string(E_calc_pr);		//E7
				send_char('\n');
			}
						
			else
			{
				send_string("m:");
				dtostrf(p_sist,2,0,sist_str);
				send_string(sist_str);
				send_char(',');
				dtostrf(p_dias,2,0,valor_m);
				send_string(valor_m);
				send_char(',');
				p_med = (p_sist+2*p_dias)/3;
				dtostrf(p_med,2,0,valor_m);
				send_string(valor_m);
		
				BPM = suma_BPM/(i_array-1);
				send_char(',');
				dtostrf(BPM,2,0,valor_m);
				send_string(valor_m);
				send_char('\n');
			}
			for (int j=0;j<indice;j++)
			{
				pres_array[j]=0;
				delta_array[j]=0;
				timer_array[j]=0;
			}
			
			BPM=0;
			fin_bp=0;
			i_array=0;
			inicio_med=0;
			indice_max=0;
			suma_BPM=0;
			min_pres=0;
			max_pres=0;
			dif_pres=0;
			max_dif=0;
			//flag_max=0;
			p_med=0;
			TCCR0B = 0x00;
			PORTB = 0b00000001;		//abrir válvula
// 			_delay_ms(5000);
// 			PORTB = 0x00;
		}
		else if (inicio_med==3)
		{
			for (int j=0;j<indice;j++)
			{
				pres_array[j]=0;
				delta_array[j]=0;
				timer_array[j]=0;
			}
			BPM=0;
			PORTB = 0b00000001;
			i_array=0;
			fin_bp=0;
			inicio_med=0;
			indice_max=0;
			suma_BPM=0;
			min_pres=0;
			max_pres=0;
			dif_pres=0;
			max_dif=0;
			//flag_max=0;
			p_med=0;
			flag_start=0;
			inicio_med=0;
			TCCR0B = 0x00;
			cont_timer=0;
			cont_timer1=0;
				
// 				while(pres_npa>3)
// 				{
// 					pres_npa=read_NPA700(NPA700_R);
// 				}
// 				PORTB = 0x00;
		}
		
	}		
}


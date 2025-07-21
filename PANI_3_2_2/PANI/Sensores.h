/*
 * Sensores.h
 *
 * Created: 22/02/2021 09:37:29 a.m.
 *  Author: Gaby Morales
 */ 


#ifndef SENSORES_H_
#define SENSORES_H_

#define FS			1875		//Full Scale sensor/0.4
#define T_ST		273.15
#define	KTE_BAR		101.325		//[kPa]
#define N_CERO		8192
#define OFFSET_PRES	0.7

int high_byte=0,low_byte=0,serialMS,serialLS;
int		vf_dt=0;
void init_mux_0(unsigned char ID_mux);
void init_mux_1(unsigned char ID_mux);
void initHAF (unsigned char ID_mux,unsigned char ID_haf);
float read_HAF(unsigned char ID_mux,unsigned char ID_haf,float temper,float atmosf,float flujo_0);
void read_NPA700est (unsigned char ID_npa);
float read_NPA700 (unsigned char ID_npa);
float read_T_NPA700 (unsigned char ID_npa);

void init_mux_0(unsigned char ID_mux)
{
	i2c_inicia_com();
	i2c_envia_dato(ID_mux);
	i2c_envia_dato(0x01);
	i2c_detener();
}

void init_mux_1(unsigned char ID_mux)
{
	i2c_inicia_com();
	i2c_envia_dato(ID_mux);
	i2c_envia_dato(0x02);
	i2c_detener();
}

void initHAF (unsigned char ID_mux,unsigned char channel,unsigned char ID_haf)
{
	i2c_inicia_com();
	i2c_envia_dato(ID_mux-1);
	i2c_envia_dato(channel);
	i2c_detener();

	i2c_inicia_com();
	i2c_envia_dato(ID_mux);
	i2c_read_nack();
	i2c_inicia_com();
	i2c_envia_dato(ID_haf);
	serialMS=(((int16_t)i2c_read_ack()<<8) | (int16_t)i2c_read_nack());
	_delay_ms(10);
	serialLS=(((int16_t)i2c_read_ack()<<8) | (int16_t)i2c_read_nack());
	i2c_detener();
}

float read_HAF(unsigned char ID_mux,unsigned char channel,unsigned char ID_haf,float temper,float atmosf,float flujo_0)
{
	float	fl_val=0,flujo_ccm=0,Q_haf=0,flujo_std=0;
	float	difer_flujo;	
	int16_t	DT_Q=0;
	
	for (vf_dt=0 ; vf_dt<1 ; vf_dt=vf_dt )
	{
		fl_val=0,flujo_ccm=0,Q_haf=0,flujo_std=0;
		DT_Q=0;

		i2c_inicia_com();
		i2c_envia_dato(ID_mux-1);
		i2c_envia_dato(channel);
		i2c_detener();

		i2c_inicia_com();
		i2c_envia_dato(ID_mux);//231
		i2c_read_nack();
		i2c_inicia_com();
		vf_dt=i2c_envia_dato(ID_haf);//147
		DT_Q=(((int16_t)i2c_read_ack()<<8) | (int16_t)i2c_read_nack());
		i2c_detener();
		if (DT_Q==0x7FFF)
		{
			vf_dt=0;
		}
	
		fl_val=(float)DT_Q;
		flujo_std=FS*((fl_val-N_CERO)/16384);		//Valor de flujo [SCCM]
		flujo_ccm=flujo_std*(T_ST+temper)*KTE_BAR/(T_ST*atmosf);	//flujo (ccm)
		Q_haf=flujo_ccm/1000;		//[lcm] litros cúbicos por minuto

		difer_flujo=Q_haf-flujo_0;
		if ((difer_flujo>7)||(difer_flujo<-7))
		{
			vf_dt=0;
			flujo_0=Q_haf;
		}
	}

	return Q_haf;
}

void read_NPA700est (unsigned char ID_npa)
{
	i2c_inicia_com();
	i2c_envia_dato(ID_npa);
	high_byte = i2c_read_ack();
	low_byte = i2c_read_nack();
	i2c_detener();
}

float read_NPA700 (unsigned char ID_npa)
{
	
	float Pr_npa=0,pres_Hg=0;
	unsigned int lectura=0;
	int h=1,sprs1=0;
	
	uint32_t acumula = 0;
	uint32_t aux_presion=0;

	lectura=0,h=1,sprs1=0;
	acumula = 0;
	aux_presion=0;
	pres_Hg=0;

  	while (h<5)
  	{
	 	i2c_inicia_com();
	 	vf_dt = i2c_envia_dato(ID_npa);
	 	lectura = (int)i2c_read_ack();
	 	lectura = lectura<<8;
	 	sprs1 = (int)i2c_read_nack();
	 	lectura = lectura|sprs1;
	 	i2c_detener();
	 	
		lectura &= 0x3FFF;

 	 	acumula += lectura;
 	 	h++;
  	}
	aux_presion=acumula/4;
	Pr_npa=0.0525*((float)aux_presion-N_CERO)+OFFSET_PRES;
	pres_Hg=0.73556*Pr_npa;
	//pres_Hg=aux_presion;
	return	pres_Hg;

}
float read_T_NPA700(unsigned char ID_npa)
{
	uint8_t Hi_P_byte=0,Lo_P_byte=0,Hi_T_byte=0,Lo_T_byte=0;
	uint16_t ent_pres=0,ent_temp=0;
	float temperatura=0;
	
	i2c_inicia_com();
	i2c_envia_dato(ID_npa);
	Hi_P_byte=i2c_read_ack();
	Lo_P_byte=i2c_read_ack();
	Hi_T_byte=i2c_read_ack();
	Lo_T_byte=i2c_read_nack();
	i2c_detener();
	ent_pres=Hi_P_byte;
	ent_pres=(ent_pres<<8|Lo_P_byte);
	ent_temp=Hi_T_byte;
	ent_temp=(ent_temp<<8|Lo_T_byte);
	ent_temp=ent_temp>>5;
	temperatura=(float)ent_temp;
	temperatura=(temperatura/10.24)-50;

	return temperatura;
}


#endif /* SENSORES_H_ */
#include "include.h"
#include "zf_common_headfile.h"
#include "encoder.h"
#include "motor.h"

LOW_Flilter ffl,fbl,fbr,ffr;
/*编码器低通滤波结果变量*/

short int num_of_time = 1;
float Current_Time = 0, Time_DeAcc = 0;

void encoder_init(uint16_t time)
{
	system_delay_ms(400);
	encoder_quad_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
	encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
	encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C25);
	encoder_quad_init(QTIMER3_ENCODER2, QTIMER3_ENCODER2_CH1_B18, QTIMER3_ENCODER2_CH2_B19);
	Flilter_init(&ffl);
	Flilter_init(&fbl);
	Flilter_init(&fbr);
	Flilter_init(&ffr);

//	system_delay_ms(500);
	
  pit_ms_init(PIT_CH0, time);                                                   // 初始化 PIT1 为周期中断 time 周期

  interrupt_set_priority(PIT_IRQn, 0);                                    // 设置 PIT1 对周期中断的中断优先级为 0
	interrupt_global_enable(0); //全局中断使能

	system_delay_ms(100);
	encoder_clear_count(ENCODER_FL);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_FR);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_BL);
	encoder_clear_count(ENCODER_BR);

	Status = Status_Stop;
}

void Get_Encoder(void)
{
	Motor2_FL.Encoder = encoder_get_count(ENCODER_FL);                  				// 获取编码器计数
	Motor3_BL.Encoder = encoder_get_count(ENCODER_BL);                          // 获取编码器计数
	Motor4_BR.Encoder = -encoder_get_count(ENCODER_BR);
	Motor1_FR.Encoder = -encoder_get_count(ENCODER_FR);
}

void Calculate_Pulse(void)
{
	Motor2_FL.Pulse_Count += Motor2_FL.Encoder;
	Motor1_FR.Pulse_Count += Motor1_FR.Encoder;
	Motor4_BR.Pulse_Count += Motor4_BR.Encoder;
	Motor3_BL.Pulse_Count += Motor3_BL.Encoder;
	
	Motor2_FL.Theta = 3.1416f * Motor2_FL.Pulse_Count / (512);
	Motor1_FR.Theta = 3.1416f * Motor1_FR.Pulse_Count / (512);
	Motor4_BR.Theta = 3.1416f * Motor4_BR.Pulse_Count / (512);
	Motor3_BL.Theta = 3.1416f * Motor3_BL.Pulse_Count / (512);


#if 0
	
	Motor2_FL.Pulse_Count = 1024 * Motor2_FL.n + Motor2_FL.Encoder;
	Motor1_FR.Pulse_Count = 1024 * Motor1_FR.n + Motor1_FR.Encoder;
	Motor4_BR.Pulse_Count = 1024 * Motor4_BR.n + Motor4_BR.Encoder;
	Motor3_BL.Pulse_Count = 1024 * Motor3_BL.n + Motor3_BL.Encoder;
	
#endif

}

void Calculate(void)
{
//	Motor1_FR.Omega_Pulse = (Motor1_FR.Pulse_Count - Motor1_FR.Pulse_Count_Former) * 100;
//	Motor2_FL.Omega_Pulse = (Motor2_FL.Pulse_Count - Motor2_FL.Pulse_Count_Former) * 100;
//	Motor3_BL.Omega_Pulse = (Motor3_BL.Pulse_Count - Motor3_BL.Pulse_Count_Former) * 100;
//	Motor4_BR.Omega_Pulse = (Motor4_BR.Pulse_Count - Motor4_BR.Pulse_Count_Former) * 100;
	Motor2_FL.Pulse_Count += Motor2_FL.Encoder;
	Motor1_FR.Pulse_Count += Motor1_FR.Encoder;
	Motor4_BR.Pulse_Count += Motor4_BR.Encoder;
	Motor3_BL.Pulse_Count += Motor3_BL.Encoder;
	
	Motor2_FL.Theta = acosf(-1.0f) * Motor2_FL.Pulse_Count / (512.0f);
	Motor1_FR.Theta = acosf(-1.0f) * Motor1_FR.Pulse_Count / (512.0f);
	Motor4_BR.Theta = acosf(-1.0f) * Motor4_BR.Pulse_Count / (512.0f);
	Motor3_BL.Theta = acosf(-1.0f) * Motor3_BL.Pulse_Count / (512.0f);

	
	Motor2_FL.Omega = 100.0f * acosf(-1.0f) * (Motor2_FL.Pulse_Count - Motor2_FL.Pulse_Count_Former) / (512.0f);
	Motor1_FR.Omega = 100.0f * acosf(-1.0f) * (Motor1_FR.Pulse_Count - Motor1_FR.Pulse_Count_Former) / (512.0f);
	Motor4_BR.Omega = 100.0f * acosf(-1.0f) * (Motor4_BR.Pulse_Count - Motor4_BR.Pulse_Count_Former) / (512.0f);
	Motor3_BL.Omega = 100.0f * acosf(-1.0f) * (Motor3_BL.Pulse_Count - Motor3_BL.Pulse_Count_Former) / (512.0f);
	
	Motor2_FL.Omega = Alpha * Motor2_FL.Omega + (1.0f - Alpha) * Motor2_FL.Omega_Former;
	Motor1_FR.Omega = Alpha * Motor1_FR.Omega + (1.0f - Alpha) * Motor1_FR.Omega_Former;
	Motor4_BR.Omega = Alpha * Motor4_BR.Omega + (1.0f - Alpha) * Motor4_BR.Omega_Former;
	Motor3_BL.Omega = Alpha * Motor3_BL.Omega + (1.0f - Alpha) * Motor3_BL.Omega_Former;//滤波
	
	Motor2_FL.Pulse_Count_Former = Motor2_FL.Pulse_Count;
	Motor1_FR.Pulse_Count_Former = Motor1_FR.Pulse_Count;
	Motor4_BR.Pulse_Count_Former = Motor4_BR.Pulse_Count;
	Motor3_BL.Pulse_Count_Former = Motor3_BL.Pulse_Count;//算角速度用

	
	Motor2_FL.Omega_Former = Motor2_FL.Omega;
	Motor1_FR.Omega_Former = Motor1_FR.Omega;
	Motor4_BR.Omega_Former = Motor4_BR.Omega;
	Motor3_BL.Omega_Former = Motor3_BL.Omega;//滤波用
	
	Get_Location_2();
	
//	Motor1_FR.Acc = (Motor1_FR.Omega - Motor1_FR.Omega_Former);//用的时候得乘100
//	Motor2_FL.Acc = (Motor2_FL.Omega - Motor2_FL.Omega_Former);
//	Motor3_BL.Acc = (Motor3_BL.Omega - Motor3_BL.Omega_Former);
//	Motor4_BR.Acc = (Motor4_BR.Omega - Motor4_BR.Omega_Former);
	
	
	
//	Motor2_FL.Pulse_Count_Former = Motor2_FL.Pulse_Count;
//	Motor1_FR.Pulse_Count_Former = Motor1_FR.Pulse_Count;
//	Motor4_BR.Pulse_Count_Former = Motor4_BR.Pulse_Count;
//	Motor3_BL.Pulse_Count_Former = Motor3_BL.Pulse_Count;//算角速度用


//	Motor2_FL.Omega_Former = Motor2_FL.Omega;
//	Motor1_FR.Omega_Former = Motor1_FR.Omega;
//	Motor4_BR.Omega_Former = Motor4_BR.Omega;
//	Motor3_BL.Omega_Former = Motor3_BL.Omega;//滤波用
	
	

#if 0
	Motor2_FL.Omega = 300 * 3.1416f * (Motor2_FL.Pulse_Count - Motor2_FL.Pulse_Count_Former) / (7 * 512 * Interrupt_Ms);
	Motor1_FR.Omega = 300 * 3.1416f * (Motor1_FR.Pulse_Count - Motor1_FR.Pulse_Count_Former) / (7 * 512 * Interrupt_Ms);
	Motor4_BR.Omega = 300 * 3.1416f * (Motor4_BR.Pulse_Count - Motor4_BR.Pulse_Count_Former) / (7 * 512 * Interrupt_Ms);
	Motor3_BL.Omega = 300 * 3.1416f * (Motor3_BL.Pulse_Count - Motor3_BL.Pulse_Count_Former) / (7 * 512 * Interrupt_Ms);

	

	
	Motor2_FL.Omega = Alpha * Motor2_FL.Omega + (1 - Alpha) * Motor2_FL.Omega_Former;
	Motor1_FR.Omega = Alpha * Motor1_FR.Omega + (1 - Alpha) * Motor1_FR.Omega_Former;
	Motor4_BR.Omega = Alpha * Motor4_BR.Omega + (1 - Alpha) * Motor4_BR.Omega_Former;
	Motor3_BL.Omega = Alpha * Motor3_BL.Omega + (1 - Alpha) * Motor3_BL.Omega_Former;//滤波
	
	Motor2_FL.Pulse_Count_Former = Motor2_FL.Pulse_Count;
	Motor1_FR.Pulse_Count_Former = Motor1_FR.Pulse_Count;
	Motor4_BR.Pulse_Count_Former = Motor4_BR.Pulse_Count;
	Motor3_BL.Pulse_Count_Former = Motor3_BL.Pulse_Count;//算角速度用


	Motor2_FL.Omega_Former = Motor2_FL.Omega;
	Motor1_FR.Omega_Former = Motor1_FR.Omega;
	Motor4_BR.Omega_Former = Motor4_BR.Omega;
	Motor3_BL.Omega_Former = Motor3_BL.Omega;//滤波用
#endif
}

void Get_n(void)
{
	if (Motor2_FL.Encoder > 1023)
	{
		encoder_clear_count(ENCODER_FL); 
		Motor2_FL.n++;
	}
	else if (Motor2_FL.Encoder < -1024)
	{
		encoder_clear_count(ENCODER_FL);
		Motor2_FL.n--;
	}
	
	if (Motor3_BL.Encoder > 1023)
	{
		encoder_clear_count(ENCODER_BL); 
		Motor3_BL.n++;
	}
	else if (Motor3_BL.Encoder < -1024)
	{
		encoder_clear_count(ENCODER_BL);
		Motor3_BL.n--;
	}
	
	if (Motor4_BR.Encoder > 1023)
	{
		encoder_clear_count(ENCODER_BR); 
		Motor4_BR.n++;
	}
	else if (Motor4_BR.Encoder < -1024)
	{
		encoder_clear_count(ENCODER_BR);
		Motor4_BR.n--;
	}
	
	if (Motor1_FR.Encoder > 1023)
	{
		encoder_clear_count(ENCODER_FR); 
		Motor1_FR.n++;
	}
	else if (Motor1_FR.Encoder < -1024)
	{
		encoder_clear_count(ENCODER_FR);
		Motor1_FR.n--;
	}
}

void Clear_Encoder(void)
{
	encoder_clear_count(ENCODER_FL);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_FR);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_BL);
	encoder_clear_count(ENCODER_BR);
}

void Flilter_init(LOW_Flilter* a)
{
	a->Last_data=0;
	a->Last_inital_data=0;
	a->Cutoff=0;
	a->Fs=0;
}

float Low_Flilter(int indata,float k,LOW_Flilter* a)
{
	  float data=0;
    data=k*(indata)+(1-k)*(a->Last_data);
    a->Last_inital_data = (float)indata;
    a->Last_data = data;
	  return data;
}

float Complementary_Flilter(float indata,float k,COMPLEMENTARY_FILTER* a)
{
	  float data=0;
    data=k*(indata)+(1-k)*(a->Last_data);
    a->Last_inital_data = indata;
    a->Last_data = data;
	  return data;
}
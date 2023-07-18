#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_FL QTIMER1_ENCODER1
#define ENCODER_BL QTIMER1_ENCODER2
#define ENCODER_BR QTIMER2_ENCODER1
#define ENCODER_FR QTIMER3_ENCODER2

#define Interrupt_Ms 1 //��˼���ж�����

#define Alpha 0.3f   //һ�׵�ͨ�˲�ϵ��




/*��ͨ�˲����ṹ��*/
typedef struct
{
    float Cutoff;           /* ��ֹƵ�� */
    float Fs;               /* ÿ��ִ�еĴ��� */
    float Last_data;        /* �ϴ��˲�������� */
    float Last_inital_data; /* �ϴ�ԭʼ���� */

}LOW_Flilter;

typedef struct
{
    float Cutoff;           /* ��ֹƵ�� */
    float Fs;               /* ÿ��ִ�еĴ��� */
    float Last_data;        /* �ϴ��˲�������� */
    float Last_inital_data; /* �ϴ�ԭʼ���� */

}COMPLEMENTARY_FILTER;


extern int sum[4];
extern float target_omega;
extern short int already_meter;
extern int a_calculate;
extern short int speed_meter;
extern float Current_Time, Time_DeAcc;
extern short int num_of_time;

extern void Clear_Encoder(void);
extern void Get_n(void);
extern void Calculate(void);
extern void Calculate_Pulse(void);
extern void Get_Encoder(void);
extern void encoder_init(unsigned short int time);
extern float Low_Flilter(int indata,float k,LOW_Flilter* a);
extern float Complementary_Flilter(float indata,float k,COMPLEMENTARY_FILTER* a);
extern void Flilter_init(LOW_Flilter* a);
#endif
#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_FL QTIMER1_ENCODER1
#define ENCODER_BL QTIMER1_ENCODER2
#define ENCODER_BR QTIMER2_ENCODER1
#define ENCODER_FR QTIMER3_ENCODER2

#define Interrupt_Ms 1 //意思是中断周期

#define Alpha 0.3f   //一阶低通滤波系数




/*低通滤波器结构体*/
typedef struct
{
    float Cutoff;           /* 截止频率 */
    float Fs;               /* 每秒执行的次数 */
    float Last_data;        /* 上次滤波后的数据 */
    float Last_inital_data; /* 上次原始数据 */

}LOW_Flilter;

typedef struct
{
    float Cutoff;           /* 截止频率 */
    float Fs;               /* 每秒执行的次数 */
    float Last_data;        /* 上次滤波后的数据 */
    float Last_inital_data; /* 上次原始数据 */

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
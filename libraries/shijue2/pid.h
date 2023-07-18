#ifndef __PID_H
#define __PID_H

#include "motor.h"

//#define Kp_Speed 0.1
//#define Ki_Speed 0.1
//#define Kd_Speed 0.1

//#define Kp_Location 0.1
//#define Ki_Location 0.1
//#define Kd_Location 0.1
#define Location_DeadZone 0.01
#define Speed_DeadZone 1

#define Standard 500

#define Max_PWM 9000
#define Max_Omega 150
#define Max_Err_Location 500
#define Max_Err_Speed 5
#define Max_Integral_Location 1000
#define Max_Integral_Speed 100000

#define FF_Correction 0//前馈系数
#define PID_Speed_Correction 1
#define PID_Four_Correction 1
#define PID_Gyro_Correction 1
#define PID_Acc_Correction 1
#define PID_Location_Correction 1

#define NULL 0
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

		
		
enum PID_MODE
{
    PID_POSITION = 0,
    PID_enhance
};


typedef  struct{

	int mode;
	
	float kp;
	float ki;
	float kd;
	
	float max_out;
	float imax_out;
	
	float set;
	float now;
	
	float out;
	float pout;
	float iout;
	float dout;

  float dbuf[3];
	float error[3];
}pid_type_def;

//extern Motor_DataTypedef Motor1_FR, Motor2_FL, Motor3_BL, Motor4_BR;
extern short int num_of_time;
extern float Average, Average_FL_BR, Average_FR_BL;


extern void PID_Gyro_PWM(void);
extern void PID_Location_All(void);
extern void Pid_Planning(void);//自己想的速度规划版本
extern void PID_Average(Motor_DataTypedef* Motor, float* num);
extern void PID_Cascade(void);
extern void Pid_Location_Plus_All(void);
extern void Pid_Location_Plus(Motor_DataTypedef* Motor);
extern void Pid_Location(Motor_DataTypedef* Motor);
extern void Pid_Speed(Motor_DataTypedef* Motor);
extern void PID_Four_Wheel(void);
extern void PID_All_Speed(void);
extern void PID_Gyro(void);
extern void PID_Gyro_Plus(void);
extern void PID_Acc_Plus(void);
extern float PID_calculate(pid_type_def*pid,float now,float set);
extern void PID_clear(pid_type_def *pid);
extern void PID_Init(pid_type_def*pid,int mode,const float PID[3],float max_out,float imax_out);
extern void stop(Motor_DataTypedef* Motor);
extern void pidsimple(Motor_DataTypedef* Motor);
extern void first_pid(Motor_DataTypedef* Motor, short int x0);
extern int pid_pid(Motor_DataTypedef* Motor,short int ordered_meter,int now_pid,int changing_speed);
#endif
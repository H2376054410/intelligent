#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR1_DIR               (D0 )
#define MOTOR1_PWM               (PWM2_MODULE3_CHA_D2)

#define MOTOR2_DIR               (D1 )
#define MOTOR2_PWM               (PWM2_MODULE3_CHB_D3)//gaodp

#define MOTOR3_DIR               (D13 )
#define MOTOR3_PWM               (PWM1_MODULE1_CHB_D15)

#define MOTOR4_DIR               (D12 )
#define MOTOR4_PWM               (PWM1_MODULE1_CHA_D14)


#define PAI acos(-1.0)
#define KK1 0.9935
#define BB1 -0.02
#define KK2 1.011
#define BB2 0.003



#define Width 0.1772f //底盘X轴上两轮中心的间距
#define Length 0.1999f //底盘Y轴上两轮中心的间距


#define r 0.0311f
#define Peng_Correction 1.36988
#define Correction 2.336725345
#define Correction_Spin 1.45f

#define Correction_Straight 1.01982f


//#define AAA 36.889f   //还得改
//#define BBB 203.25f

//#define K_FR 33.271f
//#define K_BR 31.171f
//#define K_FL 31.965f
//#define K_BL 29.963f

//#define B_FR 295.43f
//#define B_BR 411.94f
//#define B_FL 303.49f
//#define B_BL 229.57f


typedef struct
{
	short int num;//电机编号
	
	float Kp_Speed;
	float Ki_Speed;
	float Kd_Speed;
	
	float Kp_Location;
	float Ki_Location;
	float Kd_Location;
	
	float Kp_Average;
	float Ki_Average;
	float Kd_Average;
	
	float Kp_Gyro;
	float Ki_Gyro;
	float Kd_Gyro;
	
	float Kp_Acc;
	float Ki_Acc;
	float Kd_Acc;
	
	float K;
	float B;
	
	float S1;
	float S2;
	
	float Needed_Omega;//麦轮解算出的轮子需要的角速度
	int Omega_Pulse;
	int Omega_Pulse_Former;
	float Omega;//实际的角速度
	float Omega_Former;//上一次的角速度，滤波用
	float Target_Omega;
	int Target_Omega_Pulse;
	float Abs_Omega;
	float Needed_Omega_Max;
	
	float Acc;
	float Needed_Omega_Acc;
	float Needed_Omega_Pulse_Acc;
	
	short int Needed_PWM;//电机速度转化成的占空比
	short int PWM;
	
	
	float Needed_Theta;
	float Theta;
	int Needed_Pulse_Count;
	int Pulse_Count;//意思是脉冲数，即编码器一共跑了多少数字
	int Pulse_Count_Former;//意思是上一周期的数字
	int Temp_Needed_Pulse_Count;
	int Temp_Output_Pulse_Count;
	unsigned int Temp_Needed_Pulse_Count_Max;

	short int E1_Speed;
	short int E2_Speed;//速度pid中用
	int Integral_Speed;

	float E1_Location;
	float E2_Location;//位置pid中用
	float E3_Location;
	float Integral_Location;
	
	float E1_Average;
	float E2_Average;
	float E3_Average;
	float Integral_Average;
	
	float E1_Gyro;
	float E2_Gyro;
	float E3_Gyro;
	float Integral_Gyro;
	
	float E1_Acc_Y;
	float E2_Acc_Y;
	float E3_Acc_Y;
	float Integral_Acc_Y;
	
	float E1_Acc_X;
	float E2_Acc_X;
	float E3_Acc_X;
	float Integral_Acc_X;
	
	float Error_Acc_Y_temp;
	float Error_Acc_X_temp;
	float Error_Theta_temp;

	float Integral_Acc;
	
	
	float Target_Angle;
	
	short int Encoder;//编码器的值
	short int n;//记录编码器转了多少圈
	
	float temp;
	
} Motor_DataTypedef;


typedef enum
{
	Status_Stop = 0, Status_13, Status_24, Status_Spin,  Status_No_Speed
} Status_Datatypedef;


extern int all;
extern Motor_DataTypedef Motor1_FR, Motor2_FL, Motor3_BL, Motor4_BR;
extern Status_Datatypedef Status;
extern unsigned short int Flag_000, Flag_Straight_1, Flag_Straight_2, Flag_Spin_1, Flag_Spin_2, Flag_Stop, Flag_Acc, Flag_DeAcc, Flag_Finish, Flag_Straight_3, Flag_Correct, Flag_Run_State;
extern float V_X_Max, V_Y_Max, Acc_X, Acc_Y, rate, Ay_0, Ax_0, Omega_X_Max, Omega_Y_Max;
extern float Current_Location[2];
extern float Target_Angle_Acc;

extern unsigned short int Flag_Run_Normal;



extern void Go_To(float Target_X, float Target_Y);
extern void Go_Home(void);
extern void Go_To_Dot(void);
extern void Go_To_Warehouse(void);
extern void Run_Dot(void);
extern void Clear_To_Correct(void);
extern void Clear_To_Correct_Motor(Motor_DataTypedef* Motor);
extern void Open(void);
extern void Stop(void);
extern void Spin(float Theta_Angle);//Omega_Max代表车的最大转速
extern void Steady(void);//3
extern void Set_Speed_All(void);
extern void Initialize(void);
extern void Wait(void);
extern void Clear_Pulse_Count(void);
extern void Set_Location_qian_hou(float meter);
extern void Set_Location_zuo_you(float meter);
extern void Set_Speed_Wheel(Motor_DataTypedef* Motor, short int p);
extern void Set_Speed(float Vx,float Vy,float Omiga);
extern void motor_init(void);
extern void left(unsigned short int num);
extern void right(int num);
extern void forward(int num);
extern void backward(int num);
extern void Set_Location(float Rou, float Theta, float Spin);
extern void Set_Location_XOY(double X, double Y, double Acc_Max);//1
extern void Set_Location_Correction(float x, float y);
extern void stopall(void);
#endif
#include "include.h"
#include "zf_common_headfile.h"
#include "math.h"

//Motor_DataTypedef Motor1_FR = {1,    160, 1.2,  0,      2,  0.1,  0,       2,  0.5,  0,      3, 0.01f, 0,       30.532f, 291.48f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Motor_DataTypedef Motor2_FL = {2,    150, 1.0,  0,      2,  0.1,  0,       2,  0.5,  0,      0, 0, 0,       28.488f, 512.27f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Motor_DataTypedef Motor3_BL = {3,    165, 1.2,  0,      2,  0.1,  0,       2,  0.5,  0,      0, 0, 0,       27.732f, 196.69f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Motor_DataTypedef Motor4_BR = {4,    220, 1.2,  0,      2,  0.1,  0,       2,  0.5,  0,      0, 0, 0,       29.684f, 340.94f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//先速度环

	
Motor_DataTypedef Motor1_FR = {1,    110, 1.8,  0,     2,  0.02,  0,       2,  0.5,  0,      3, 0.02, 0,   0, 0, 0,     30.532f, 291.48f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Motor_DataTypedef Motor2_FL = {2,    100, 1.5,  0,    2,  0.02,  0,       2,  0.5,  0,      0, 0, 0,     0, 0, 0,   28.488f, 512.27f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Motor_DataTypedef Motor3_BL = {3,  	 70, 1.5,  0,     2,  0.02,  0,       2,  0.5,  0,      0, 0, 0,     0, 0, 0,   27.732f, 196.69f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Motor_DataTypedef Motor4_BR = {4,    100, 1.9,  0,      2,  0.02,  0,       2,  0.5,  0,      0, 0, 0,     0, 0, 0,   29.684f, 340.94f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//先速度环

Status_Datatypedef Status = Status_Stop;
Status_Datatypedef Status_Correction = Status_Stop;
	
float Acc = 0, Acc_X = 0, Acc_Y = 0, V_X_Max = 0, V_Y_Max = 0, Omega_X_Max = 0, Omega_Y_Max = 0, rate = 0, Ay_0, Ax_0;
unsigned short int Flag_000 = 0, Flag_Straight_1 = 0, Flag_Straight_2 = 0, Flag_Spin_1 = 0, Flag_Spin_2 = 0, Flag_Stop = 0, Flag_Acc = 0, Flag_DeAcc = 0, Flag_Finish = 0, Flag_Straight_3 = 0, Flag_Correct = 1;
unsigned short int Flag_Run_State = 1;
float Target_Angle_Acc = 0;
int flag_dot = 0;
float Current_Location[2] = {0};
float Flag_If_Dot_Gone[30] = {0};


unsigned short int Flag_Run_Normal = 1;



double g_1(double theta)
{
	return 0.7981*theta*theta*theta*theta*theta*theta - 4.131*theta*theta*theta*theta*theta + 8.437*theta*theta*theta*theta - 8.414*theta*theta*theta + 4.114*theta*theta + 0.1139*theta + 4.329e-15;
}

double f(double theta)//这两个暂时不定，可以继续改
{
	return 0.5706*theta*theta*theta*theta*theta - 2.119*theta*theta*theta*theta + 2.61*theta*theta*theta - 1.169*theta*theta + 0.1705*theta + 1.967;
}

void XY_Correction(double* a, double* b)
{
	unsigned short int quadrant;
	double X0 = *a, Y0 = *b, theta0, k, X_In, Y_In;

	if (X0 >= 0 && Y0 >= 0)
		quadrant = 1;
	else if (X0 < 0 && Y0 > 0)
		quadrant = 2;
	else if (X0 <= 0 && Y0 <= 0)
		quadrant = 3;
	else if (X0 > 0 && Y0 < 0)
		quadrant = 4;

	switch (quadrant)
	{
		case 1:
			if (X0 == 0)
			{
				//printf(">?");
				*a = 0;
				*b = (Y0 - BB2)/KK2;
			}
			else
			{
				theta0 = g_1(atan(Y0/X0));
				k = tan(theta0);
				X_In = (2 - ((0.5*PAI*sqrt(X0*X0 + Y0*Y0) - 0.5*PAI*f(theta0))/(theta0*KK2 - theta0*KK1 - 0.5*PAI*KK1)))/(sqrt(1 + k*k));
				Y_In = k * X_In;
				*a = X_In;
				*b = Y_In;
			}
			break;
		case 2:
			X0 = -X0;
			theta0 = g_1(atan(Y0/X0));
			k = tan(theta0);
			X_In = (2 - ((0.5*PAI*sqrt(X0*X0 + Y0*Y0) - 0.5*PAI*f(theta0))/(theta0*KK2 - theta0*KK1 - 0.5*PAI*KK1)))/(sqrt(1 + k*k));
			Y_In = k * X_In;
			*a = -X_In;
			*b = Y_In;
			break;
		case 3:
			X0 = -X0;
			Y0 = -Y0;
			if (X0 == 0)
			{
				//printf(">?");
				*a = 0;
				*b = -(Y0 - BB2)/KK2;
			}
			else
			{
				theta0 = g_1(atan(Y0/X0));
				k = tan(theta0);
				X_In = (2 - ((0.5*PAI*sqrt(X0*X0 + Y0*Y0) - 0.5*PAI*f(theta0))/(theta0*KK2 - theta0*KK1 - 0.5*PAI*KK1)))/(sqrt(1 + k*k));
				Y_In = k * X_In;
				*a = -X_In;
				if (*b != 0)
					*b = -Y_In;
				else
					*b = 0;
			}
			break;
		case 4:
			Y0 = -Y0;	
			theta0 = g_1(atan(Y0/X0));
			k = tan(theta0);
			X_In = (2 - ((0.5*PAI*sqrt(X0*X0 + Y0*Y0) - 0.5*PAI*f(theta0))/(theta0*KK2 - theta0*KK1 - 0.5*PAI*KK1)))/(sqrt(1 + k*k));
			Y_In = k * X_In;
			*a = X_In;
			*b = -Y_In;
			break;
	}
}



void Run_Dot(void)
{
	//system_delay_ms(3000);
	receive_finished(0x4D);
	printf("我已发送M");
	while (!second_flag)
	{}
	while (1)
	{
		if(situation==2)//跑点&拿图片&放仓库
		{
			if(motion_situation==0)
			{
				if(send_time==0)
				{
					receive_finished(0x43);
					printf("\r\n我已发送C\r\n");//识别图片
					send_time=1;
				}
				while (!flag_recognize_finished)
				{}
				if (flag_recognize_finished)
				{
					Servo_Run();
					printf("697");
					Go_To_Warehouse();
					printf("67786");
					flag_dot++;
					if (flag_dot == flag)//这里本来是flag_dot == flag
						break;
				}
				
			}
			else if (second_flag)
			{
				printf("im here!, %f, %f\n", 0.2 * (zuobiaodui[0][0] - 1), 0.2 * (zuobiaodui[0][1] - 1));
				Go_To_Dot();
				motion_situation=0;					
			}	
		}
	}
	Initialize();
}

void Go_Home(void)
{
	Open();
	Flag_Run_State = 0;//置0的话没有位姿矫正
	printf("current loc = %f, %f\n", 1+5*Current_Location[0], 1+5*Current_Location[1]);
	Set_Location_XOY(-Current_Location[0], -Current_Location[1], 1);
	Wait();
}

void Go_To_Dot(void)
{
	float distance = 1000;
	float distance_temp = 0;
	int chosen = 0;
	for (int t = flag;t > 0;t--)
	{
		if (!Flag_If_Dot_Gone[t-1])
		{
			distance_temp = sqrt((zuobiaodui[t-1][0] - Current_Location[0]) * (zuobiaodui[t-1][0] - Current_Location[0]) + (zuobiaodui[t-1][1] - Current_Location[1]) * (zuobiaodui[t-1][1] - Current_Location[1]));
			if (distance_temp < distance)
			{
				distance = distance_temp;
				chosen = t-1;
			}
			printf("chosen = %d\n", chosen);
		}
	}
	
	Open();
	Set_Location_XOY(zuobiaodui[chosen][0]-Current_Location[0], zuobiaodui[chosen][1]-Current_Location[1], 1);
	printf("目标：%f, %f\n", 1+5*zuobiaodui[chosen][0], 1+5*zuobiaodui[chosen][1]);
	printf("现在是运动时间");
	Wait();
	Flag_If_Dot_Gone[chosen] = 1;
	Current_Location[0] = zuobiaodui[chosen][0];
	Current_Location[1] = zuobiaodui[chosen][1];//更新实时坐标
}


void Go_To_Warehouse(void)
{
	Open();
	Flag_Run_State = 0;//置0的话没有位姿矫正
	printf("current loc = %f, %f\n", 1+5*Current_Location[0], 1+5*Current_Location[1]);
	switch ((int)image_kind)
	{
		case 0:
			Set_Location_XOY(-0.3-Current_Location[0], 0, 1);
			Wait();
			putttt();
			Current_Location[0] = -0.3;
			printf("\r\n %d \r\n",0);
			break;
		case 1:
			Set_Location_XOY(0, 5-Current_Location[1], 1);
			Wait();
			putttt();
			Current_Location[1] = 5;
		  printf("\r\n %d \r\n",1);
			break;
		case 2:
			Set_Location_XOY(7.1-Current_Location[0], 0, 1);
			Wait();
			putttt();
			Current_Location[0] = 7.1;
		  printf("\r\n %d \r\n",2);
			break;
		case 3:
			Set_Location_XOY(0, -0.2-Current_Location[1], 1);
			Wait();
			putttt();
			Current_Location[1] = -0.2;
		  printf("\r\n %d \r\n",3);
			break;
		case 4:
			fangche();
			break;
		case -1:
			fangche();
			break;
		default:
			fangche();
		  printf("\r\n default \r\n");
		  break;

	}
	motion_situation = 1;
	printf("now,current loc = %f, %f\n", 1+5*Current_Location[0], 1+5*Current_Location[1]);
}



void Stop_All(void)
{
	Set_Speed_Wheel(&Motor2_FL,0);
	Set_Speed_Wheel(&Motor1_FR,0);
	Set_Speed_Wheel(&Motor3_BL,0);
	Set_Speed_Wheel(&Motor4_BR,0);
}


void Set_Speed_Wheel(Motor_DataTypedef* Motor, short int p)//p是占空比，最大10000
{
	if (Motor->num == 1)
	{
		if (p < 0)
		{
			gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                         // DIR输出高电平
			pwm_set_duty(MOTOR1_PWM, -p);
		}
		else
		{
			gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                          // DIR输出低电平
			pwm_set_duty(MOTOR1_PWM, p);
		}
	}
	
	else if (Motor->num == 2)
	{
		if (p < 0)
		{
			gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                         // DIR输出高电平
			pwm_set_duty(MOTOR2_PWM, -p);
		}
		else
		{
			gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                          // DIR输出低电平
			pwm_set_duty(MOTOR2_PWM, p);
		}
	}
	
	else if (Motor->num == 3)
	{
		if (p < 0)
		{
			gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                         // DIR输出高电平
			pwm_set_duty(MOTOR3_PWM, -p);
		}
		else
		{
			gpio_set_level(MOTOR3_DIR, GPIO_LOW);                                          // DIR输出低电平
			pwm_set_duty(MOTOR3_PWM, p);
		}
	}
	
	else if (Motor->num == 4)
	{
		if (p < 0)
		{
			gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                         // DIR输出高电平
			pwm_set_duty(MOTOR4_PWM, -p);
		}
		else
		{
			gpio_set_level(MOTOR4_DIR, GPIO_LOW);                                          // DIR输出低电平
			pwm_set_duty(MOTOR4_PWM, p);
		}
	}

}

void Clean(Motor_DataTypedef* Motor)
{
	Motor->Abs_Omega = 0;
	Motor->Acc = 0;
	Motor->E1_Average = 0;
	Motor->E1_Location = 0;
	Motor->E1_Speed = 0;
	Motor->E1_Gyro = 0;
	Motor->E2_Average = 0;
	Motor->E2_Location = 0;
	Motor->E2_Speed = 0;
	Motor->E2_Gyro = 0;
	Motor->E3_Average = 0;
	Motor->E3_Location = 0;
	Motor->E3_Gyro = 0;
	Motor->Encoder = 0;
	Motor->Integral_Average = 0;
	Motor->Integral_Location = 0;
	Motor->Integral_Speed = 0;
	Motor->Integral_Gyro = 0;
	Motor->n = 0;
	Motor->Needed_Omega = 0;
	Motor->Needed_Omega_Acc = 0;
	Motor->Needed_Omega_Max = 0;
	Motor->Needed_Omega_Pulse_Acc = 0;
	Motor->Needed_Pulse_Count = 0;
	Motor->Needed_PWM = 0;
	Motor->Needed_Theta = 0;
	Motor->Omega = 0;
	Motor->Omega_Former = 0;
	Motor->Omega_Pulse = 0;
	Motor->Omega_Pulse_Former = 0;
	Motor->Pulse_Count = 0;
	Motor->Pulse_Count_Former = 0;
	Motor->PWM = 0;
	Motor->S1 = 0;
	Motor->S2 = 0;
	Motor->Target_Omega = 0;
	Motor->Target_Omega_Pulse = 0;
	Motor->temp = 0;
	Motor->Temp_Needed_Pulse_Count = 0;
	Motor->Temp_Needed_Pulse_Count_Max = 0;
	Motor->Temp_Output_Pulse_Count = 0;
	Motor->Theta = 0;
	//Motor->Target_Angle = 0;
	Motor->E1_Acc_Y = 0;
	Motor->E2_Acc_Y = 0;
	Motor->E3_Acc_Y = 0;
	Motor->Integral_Acc_Y = 0;
	
	Motor->E1_Acc_X = 0;
	Motor->E2_Acc_X = 0;
	Motor->E3_Acc_X = 0;
	Motor->Integral_Acc_X = 0;
	
	Motor->Error_Acc_Y_temp = 0;
	Motor->Error_Acc_X_temp = 0;
	Motor->Error_Theta_temp = 0;

	Motor->Integral_Acc = 0;

}

void Clear_To_Correct(void)
{
	Clear_To_Correct_Motor(&Motor1_FR);
	Clear_To_Correct_Motor(&Motor2_FL);
	Clear_To_Correct_Motor(&Motor3_BL);
	Clear_To_Correct_Motor(&Motor4_BR);
}

void Clear_To_Correct_Motor(Motor_DataTypedef* Motor)
{
	Motor->E1_Average = 0;
	Motor->E1_Location = 0;
	Motor->E1_Speed = 0;
	Motor->E2_Average = 0;
	Motor->E2_Location = 0;
	Motor->E2_Speed = 0;
	Motor->E3_Average = 0;
	Motor->E3_Location = 0;
	Motor->Encoder = 0;
	Motor->Integral_Average = 0;
	Motor->Integral_Location = 0;
	Motor->Integral_Speed = 0;
	Motor->Needed_Omega = 0;
	Motor->Needed_Omega_Acc = 0;
	Motor->Needed_Omega_Max = 0;
	Motor->Needed_Omega_Pulse_Acc = 0;
	Motor->Needed_Pulse_Count = 0;
	Motor->Needed_PWM = 0;
	Motor->Needed_Theta = 0;
	Motor->Omega = 0;
	Motor->Omega_Former = 0;
	Motor->Omega_Pulse = 0;
	Motor->Omega_Pulse_Former = 0;
	Motor->Pulse_Count = 0;
	Motor->Pulse_Count_Former = 0;
	Motor->PWM = 0;
	Motor->S1 = 0;
	Motor->S2 = 0;
	Motor->Target_Omega = 0;
	Motor->Target_Omega_Pulse = 0;
	Motor->Theta = 0;
}

void Initialize(void)
{
	Status = Status_Stop;
	Clean(&Motor1_FR);
	Clean(&Motor2_FL);
	Clean(&Motor3_BL);
	Clean(&Motor4_BR);
	
	encoder_clear_count(QTIMER1_ENCODER1);                                       // 清空编码器计数
	encoder_clear_count(QTIMER1_ENCODER2);                                       // 清空编码器计数
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER3_ENCODER2);
	
	num_of_time = 0;
	Current_Time = 0;
	Time_DeAcc = 0;
	Average = 0;
	Average_FL_BR = 0;
	Average_FR_BL = 0;
	Acc = 0;
	Acc_X = 0;
	Acc_Y = 0;
	V_X_Max = 0;
	V_Y_Max = 0;
	rate = 0;
	
	Flag_Straight_1 = 0;
	Flag_Straight_2 = 0; 
	Flag_Spin_1 = 0;
	Flag_Spin_2 = 0; 
	Flag_Stop = 0;
	Flag_Acc = 0;
	Flag_DeAcc = 0;
	Flag_Finish = 0;
	Flag_Straight_3 = 0;
	Flag_Correct = 1;
	Flag_Run_State = 1;
	
	z_degree = 0;
	
	change_position[0] = 0;
	change_position[1] = 0;
	Flag_If_Received_Picture_Dot = 0;
	
	pwm_set_duty(MOTOR1_PWM, 0);
	pwm_set_duty(MOTOR2_PWM, 0);
	pwm_set_duty(MOTOR3_PWM, 0);
	pwm_set_duty(MOTOR4_PWM, 0);

}

void Wait(void)//如何判定它走完了？
{
	while (1)
	{
		if (Flag_Finish)
		{
			Stop();
			Initialize();
			system_delay_ms(500);	
			break;
		}
	}
}

void Set_Speed_All(void)
{
	Set_Speed_Wheel(&Motor2_FL, Motor2_FL.PWM);
	Set_Speed_Wheel(&Motor1_FR, Motor1_FR.PWM);
	Set_Speed_Wheel(&Motor3_BL, Motor3_BL.PWM);
	Set_Speed_Wheel(&Motor4_BR, Motor4_BR.PWM);

}

//void Set_Location_qian_hou(float meter)
//{
//	Motor2_FL.Needed_Pulse_Count = 6730 * meter;
//	Motor1_FR.Needed_Pulse_Count = 6730 * meter;
//	Motor3_BL.Needed_Pulse_Count = 6730 * meter;
//	Motor4_BR.Needed_Pulse_Count = 6730 * meter;
//}
//void Set_Location_zuo_you(float meter)//正左负右
//{
//	Motor2_FL.Needed_Pulse_Count = 6730 * -meter;
//	Motor4_BR.Needed_Pulse_Count = 6730 * -meter;
//	Motor1_FR.Needed_Pulse_Count = 6730 * meter;
//	Motor3_BL.Needed_Pulse_Count = 6730 * meter;
//}

void Set_Location(float Rou, float Theta, float Spin)//目前还不能用，也好像用不到
{
//	if (Theta == 3.14f || Theta == 6.28f || Theta == 9.42f)
//		Status = Straight;
//	else
//	{
//		Status = Diagonal;
//		rate = fabs(Rou * (sinf(Theta) - cosf(Theta))) / fabs(Rou * (sinf(Theta) + cosf(Theta)));
//	}
	Motor1_FR.Needed_Pulse_Count = Correction * 512 * (Rou * (sinf(Theta) - cosf(Theta)) + Spin * (Width + Length) * 0.5f) / (r * 3.1416f);
	Motor2_FL.Needed_Pulse_Count = Correction * 512 * (Rou * (sinf(Theta) + cosf(Theta)) - Spin * (Width + Length) * 0.5f) / (r * 3.1416f);
	Motor3_BL.Needed_Pulse_Count = Correction * 512 * (Rou * (sinf(Theta) - cosf(Theta)) - Spin * (Width + Length) * 0.5f) / (r * 3.1416f);
	Motor4_BR.Needed_Pulse_Count = Correction * 512 * (Rou * (sinf(Theta) + cosf(Theta)) + Spin * (Width + Length) * 0.5f) / (r * 3.1416f);
}


void Open(void)
{
	Flag_000 = 1;
}

void Stop(void)
{
	Flag_000 = 0;
}

void Set_Location_Correction(float x, float y)
{
	Flag_Run_Normal = 0;
	//printf("x = %f, y = %f\n", x, y);
//	x /= 730.0f;
//	y /= 730.0f;
//	Target_X = x / 0.2f;
//	Target_Y = y / 0.2f;
	
	if (x == 0 && y == 0)
	{
		printf("Correction error\n");
	}
	else
	{
	
		if (x * y >= 0)
		{
			Status = Status_13;
			rate = (y - Correction_Straight * x) / (y + Correction_Straight * x);
		}
		else
		{
			Status = Status_24;
			rate = (y + Correction_Straight * x) / (y - Correction_Straight * x);
		}
		//	if (fabs(x) >= fabs(y))
		//	{
		//		Flag_X_or_Y = 0;
		//		Rate_X_Y = y / x;
		//	}
		//	else
		//	{
		//		Flag_X_or_Y = 1;
		//		Rate_X_Y = x / y;
		//	}

		Motor1_FR.Needed_Theta = Correction * (y - Correction_Straight * x) / (r);
		Motor2_FL.Needed_Theta = Correction * (y + Correction_Straight * x) / (r);
		Motor3_BL.Needed_Theta = Correction * (y - Correction_Straight * x) / (r);
		Motor4_BR.Needed_Theta = Correction * (y + Correction_Straight * x) / (r);
	}
//		if (Status == Status_13 && Flag_X_or_Y == 0)
//		{
//			Rate_Temp = Motor2_FL.Needed_Theta / Target_X;
//		}
//		else if (Status == Status_13 && Flag_X_or_Y == 1)
//		{
//			Rate_Temp = Motor2_FL.Needed_Theta / Target_Y;
//		}
//		else if (Status == Status_24 && Flag_X_or_Y == 0)
//		{
//			Rate_Temp = Motor1_FR.Needed_Theta / Target_X;
//		}
//		else if (Status == Status_24 && Flag_X_or_Y == 1)
//		{
//			Rate_Temp = Motor1_FR.Needed_Theta / Target_Y;
//		}
}


void Go_To(float Target_X, float Target_Y)
{
	Open();
	Flag_Run_State = 0;
	Set_Location_XOY(0.2f * (Target_X - Current_X), 0.2f * (Target_Y - Current_Y), 0.7f);
	Wait();
}



void Set_Location_XOY(double X, double Y, double Acc_Max)//1
{
#if 1
	if (X * Y >= 0)
	{
		Status = Status_13;
		rate = (Y - Correction_Straight * X) / (Y + Correction_Straight * X);
	}
	else
	{
		Status = Status_24;
		rate = (Y + Correction_Straight * X) / (Y - Correction_Straight * X);
	}


	if (fabs(X) < 0.2f && fabs(Y) < 0.2f)//这里需要测试
	{
		Flag_Run_Normal = 0;
		
		Motor1_FR.Needed_Theta = Correction * (Y - Correction_Straight * X) / (r);
		Motor2_FL.Needed_Theta = Correction * (Y + Correction_Straight * X) / (r);
		Motor3_BL.Needed_Theta = Correction * (Y - Correction_Straight * X) / (r);
		Motor4_BR.Needed_Theta = Correction * (Y + Correction_Straight * X) / (r);

		
		Flag_Straight_2 = 1;
	}
	else
	{
		Flag_Run_Normal = 1;
		Acc_X = Acc_Max * X / sqrt(X * X + Y * Y);
		Acc_Y = Acc_Max * Y / sqrt(X * X + Y * Y);

		if (X == 0)
			V_X_Max = 0;
		else
			V_X_Max = (X / fabs(X)) * sqrt(X * Acc_X);
		
		if (Y == 0)
			V_Y_Max = 0;
		else
			V_Y_Max = (Y / fabs(Y)) * sqrt(Y * Acc_Y);
		
		Motor1_FR.Needed_Omega_Acc = Correction * (Acc_Y - Correction_Straight * Acc_X) / (r);
		Motor2_FL.Needed_Omega_Acc = Correction * (Acc_Y + Correction_Straight * Acc_X) / (r);
		Motor3_BL.Needed_Omega_Acc = Correction * (Acc_Y - Correction_Straight * Acc_X) / (r);
		Motor4_BR.Needed_Omega_Acc = Correction * (Acc_Y + Correction_Straight * Acc_X) / (r);
		
		Motor1_FR.Needed_Omega_Max = Correction * (V_Y_Max - Correction_Straight * V_X_Max) / (r);
		Motor2_FL.Needed_Omega_Max = Correction * (V_Y_Max + Correction_Straight * V_X_Max) / (r);
		Motor3_BL.Needed_Omega_Max = Correction * (V_Y_Max - Correction_Straight * V_X_Max) / (r);
		Motor4_BR.Needed_Omega_Max = Correction * (V_Y_Max + Correction_Straight * V_X_Max) / (r);
		
				
		Motor1_FR.Needed_Theta = Correction * (Y - Correction_Straight * X) / (r);
		Motor2_FL.Needed_Theta = Correction * (Y + Correction_Straight * X) / (r);
		Motor3_BL.Needed_Theta = Correction * (Y - Correction_Straight * X) / (r);
		Motor4_BR.Needed_Theta = Correction * (Y + Correction_Straight * X) / (r);
		
		Motor1_FR.S1 = 0.5f * Motor1_FR.Needed_Omega_Max * Motor1_FR.Needed_Omega_Max / Motor1_FR.Needed_Omega_Acc;
		Motor2_FL.S1 = 0.5f * Motor2_FL.Needed_Omega_Max * Motor2_FL.Needed_Omega_Max / Motor2_FL.Needed_Omega_Acc;
		Motor3_BL.S1 = 0.5f * Motor3_BL.Needed_Omega_Max * Motor3_BL.Needed_Omega_Max / Motor3_BL.Needed_Omega_Acc;
		Motor4_BR.S1 = 0.5f * Motor4_BR.Needed_Omega_Max * Motor4_BR.Needed_Omega_Max / Motor4_BR.Needed_Omega_Acc;

		Flag_Straight_1 = 1;
		Flag_Acc = 1;

	}
		
#endif
}


void Set_Location_Decelerate(float x)
{
//		Target_X = X/0.2f;
//		Target_Y = Y/0.2f;
//		if (fabs(X) >= fabs(Y))
//		{
//			Flag_X_or_Y = 0;
//			Rate_X_Y = Y / X;
//		}
//		else
//		{
//			Flag_X_or_Y = 1;
//			Rate_X_Y = X / Y;
//		}
////		Currtent_X_Temp = X/0.2f;
////		Currtent_Y_Temp = Y/0.2f;//换算成坐标点
////		if (X * Y >= 0)
////		{
////			Rate_Temp = (Y - X) / (Y + X);
////		}
////		else
////		{
////			Rate_Temp = (Y + X) / (Y - X);
////		}//求出比值

//		if (fabs(Y/X) < 0.17)
//		{
//			Y += fabs(X/70);
//		}
//		XY_Correction(&X, &Y);
//		if (X * Y >= 0)
//		{
//			Status = Status_13;
//			rate = (Y - X) / (Y + X);
//		}
//		else
//		{
//			Status = Status_24;
//			rate = (Y + X) / (Y - X);
//		}

//		//Target_Angle_Acc = atan2(Y, X);
//		
//		Ay_0 = Y / (X * X + Y * Y);
//		Ax_0 = X / (X * X + Y * Y);
//		
//		Flag_Straight_1 = 1;
//		
//		V_X_Max = Vmax * X / sqrt(X * X + Y * Y);//sin，cos
//		V_Y_Max = Vmax * Y / sqrt(X * X + Y * Y);
//		Acc_X = Vmax * Vmax * X /(X * X + Y * Y - Vmax * t * sqrt(X * X + Y * Y));
//		Acc_Y = Vmax * Vmax * Y /(X * X + Y * Y - Vmax * t * sqrt(X * X + Y * Y));
//		
//		Motor1_FR.Needed_Omega_Acc = Correction * (Acc_Y - Acc_X) / (r);
//		Motor2_FL.Needed_Omega_Acc = Correction * (Acc_Y + Acc_X) / (r);
//		Motor3_BL.Needed_Omega_Acc = Correction * (Acc_Y - Acc_X) / (r);
//		Motor4_BR.Needed_Omega_Acc = Correction * (Acc_Y + Acc_X) / (r);
//		
//		Motor1_FR.Needed_Omega_Max = Correction * (V_Y_Max - V_X_Max) / (r);
//		Motor2_FL.Needed_Omega_Max = Correction * (V_Y_Max + V_X_Max) / (r);
//		Motor3_BL.Needed_Omega_Max = Correction * (V_Y_Max - V_X_Max) / (r);
//		Motor4_BR.Needed_Omega_Max = Correction * (V_Y_Max + V_X_Max) / (r);

//		
//		Motor1_FR.Needed_Theta = Correction * (Y - X) / (r);
//		Motor2_FL.Needed_Theta = Correction * (Y + X) / (r);
//		Motor3_BL.Needed_Theta = Correction * (Y - X) / (r);
//		Motor4_BR.Needed_Theta = Correction * (Y + X) / (r);
//		
//		Motor1_FR.S1 = 0.5f * Motor1_FR.Needed_Omega_Max * Motor1_FR.Needed_Omega_Max / Motor1_FR.Needed_Omega_Acc;
//		Motor2_FL.S1 = 0.5f * Motor2_FL.Needed_Omega_Max * Motor2_FL.Needed_Omega_Max / Motor2_FL.Needed_Omega_Acc;
//		Motor3_BL.S1 = 0.5f * Motor3_BL.Needed_Omega_Max * Motor3_BL.Needed_Omega_Max / Motor3_BL.Needed_Omega_Acc;
//		Motor4_BR.S1 = 0.5f * Motor4_BR.Needed_Omega_Max * Motor4_BR.Needed_Omega_Max / Motor4_BR.Needed_Omega_Acc;

//		if (Status == Status_13 && Flag_X_or_Y == 0)
//		{
//			Rate_Temp = Motor2_FL.Needed_Theta / Target_X;
//		}
//		else if (Status == Status_13 && Flag_X_or_Y == 1)
//		{
//			Rate_Temp = Motor2_FL.Needed_Theta / Target_Y;
//		}
//		else if (Status == Status_24 && Flag_X_or_Y == 0)
//		{
//			Rate_Temp = Motor1_FR.Needed_Theta / Target_X;
//		}
//		else if (Status == Status_24 && Flag_X_or_Y == 1)
//		{
//			Rate_Temp = Motor1_FR.Needed_Theta / Target_Y;
//		}

//	
//	Flag_Acc = 1;

}

void Spin(float Theta_Angle)//Omega_Max代表车的最大转速,2
{
#if 0
//t = Theta_Angle/30
//t/2 = Theta_Angle/60
//omega_max = 60 * Motor1_FR.Needed_Theta / Theta_Angle
//Needed_Omega_Acc = 60*omega_max/Theta_Angle
		
		Status = Status_Spin;
		Flag_Spin_1 = 1;
		Motor1_FR.Target_Angle += Theta_Angle;

		Motor1_FR.Needed_Theta = 89.0f * Theta_Angle / 360.0f;
		Motor2_FL.Needed_Theta = -Motor1_FR.Needed_Theta;
		Motor3_BL.Needed_Theta = -Motor1_FR.Needed_Theta;
		Motor4_BR.Needed_Theta = Motor1_FR.Needed_Theta;
	
		Motor1_FR.Needed_Omega_Max = 240.0f * fabs(Motor1_FR.Needed_Theta) / Theta_Angle;
		Motor2_FL.Needed_Omega_Max = -Motor1_FR.Needed_Omega_Max;
		Motor3_BL.Needed_Omega_Max = -Motor1_FR.Needed_Omega_Max;
		Motor4_BR.Needed_Omega_Max = Motor1_FR.Needed_Omega_Max;


		
		Motor1_FR.Needed_Omega_Acc = 240.0f * fabs(Motor1_FR.Needed_Omega_Max) / Theta_Angle;
		Motor2_FL.Needed_Omega_Acc = -Motor1_FR.Needed_Omega_Acc;
		Motor3_BL.Needed_Omega_Acc = -Motor1_FR.Needed_Omega_Acc;
		Motor4_BR.Needed_Omega_Acc = Motor1_FR.Needed_Omega_Acc;
		
			
		Motor1_FR.S1 = 0.5f * Motor1_FR.Needed_Theta;
		Motor2_FL.S1 = -Motor1_FR.S1;
		Motor3_BL.S1 = -Motor1_FR.S1;
		Motor4_BR.S1 = Motor1_FR.S1;

	Flag_Acc = 1;
#endif

#if 1
	Status = Status_Spin;
	Flag_Spin_1 = 1;
	Motor1_FR.Target_Angle += Theta_Angle;
	
	Motor1_FR.Target_Omega = 30.0f * (Theta_Angle / fabs(Theta_Angle));
	Motor2_FL.Target_Omega = -Motor1_FR.Target_Omega;
	Motor3_BL.Target_Omega = -Motor1_FR.Target_Omega;
	Motor4_BR.Target_Omega = Motor1_FR.Target_Omega;

#endif
}

void Steady(void)//3
{
	Status = Status_Stop;
	Flag_Stop = 1;
	Motor1_FR.Target_Angle = 0;
}


void Set_Speed(float Vx, float Vy, float Omiga)//输入小车需要的速度
{
	Motor1_FR.Needed_Omega = (Vy - Vx + Omiga * (Width + Length) * 0.5f) / r;
	Motor2_FL.Needed_Omega = (Vy + Vx - Omiga * (Width + Length) * 0.5f) / r;
	Motor3_BL.Needed_Omega = (Vy - Vx - Omiga * (Width + Length) * 0.5f) / r;
	Motor4_BR.Needed_Omega = (Vy + Vx + Omiga * (Width + Length) * 0.5f) / r;//转化为每个轮子的角速度

	Motor1_FR.Needed_PWM = Motor1_FR.Needed_Omega * Motor1_FR.K + Motor1_FR.B;
	Motor2_FL.Needed_PWM = Motor2_FL.Needed_Omega * Motor2_FL.K + Motor2_FL.B;
	Motor3_BL.Needed_PWM = Motor3_BL.Needed_Omega * Motor3_BL.K + Motor3_BL.B;
	Motor4_BR.Needed_PWM = Motor4_BR.Needed_Omega * Motor4_BR.K + Motor4_BR.B;//转化为每个轮子需要的占空比
	
	Motor1_FR.PWM = Motor1_FR.Needed_PWM;
	Motor2_FL.PWM = Motor2_FL.Needed_PWM;
	Motor3_BL.PWM = Motor3_BL.Needed_PWM;
	Motor4_BR.PWM = Motor4_BR.Needed_PWM;//给pid调的实际pwm赋初始值
	
	if (Motor1_FR.Needed_PWM > 0)
	{
		gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                         // DIR输出高电平
		pwm_set_duty(MOTOR1_PWM, Motor1_FR.Needed_PWM);
	}
	else
	{
		gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                          // DIR输出低电平
		pwm_set_duty(MOTOR1_PWM, -Motor1_FR.Needed_PWM);
	}
	
	if (Motor2_FL.Needed_PWM > 0)
	{
		gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                         // DIR输出高电平
		pwm_set_duty(MOTOR2_PWM, Motor2_FL.Needed_PWM);
	}
	else
	{
		gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                          // DIR输出低电平
		pwm_set_duty(MOTOR2_PWM, -Motor2_FL.Needed_PWM);
	}
	
	if (Motor3_BL.Needed_PWM > 0)
	{
		gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                         // DIR输出高电平
		pwm_set_duty(MOTOR3_PWM, Motor3_BL.Needed_PWM);
	}
	else
	{
		gpio_set_level(MOTOR3_DIR, GPIO_LOW);                                          // DIR输出低电平
		pwm_set_duty(MOTOR3_PWM, -Motor3_BL.Needed_PWM);
	}
	
	if (Motor4_BR.Needed_PWM > 0)
	{
		gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                         // DIR输出高电平
		pwm_set_duty(MOTOR4_PWM, Motor4_BR.Needed_PWM);
	}
	else
	{
		gpio_set_level(MOTOR4_DIR, GPIO_LOW);                                          // DIR输出低电平
		pwm_set_duty(MOTOR4_PWM, -Motor4_BR.Needed_PWM);
	}	
}

void Turn_Left_90(float w)
{
	Set_Speed(0, 0, -w);
}


void left(uint16_t num)
{
	Set_Speed(-num, 0, 0);
}

void right(int num)
{
	Set_Speed(num, 0, 0);
}

void forward(int num)
{
	Set_Speed(0, num, 0);
}

void backward(int num)
{
	Set_Speed(0, -num, 0);
}


void motor_init(void)
{
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
	pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
	
	gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
	pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

	gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
	pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

	gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
	pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
	
	pwm_init(dianci_gpio, SERVO_MOTOR_FREQ, 0);
	pwm_init(PWM4_MODULE2_CHA_C30, SERVO_MOTOR_FREQ, 0);
	pwm_init(PWM4_MODULE3_CHA_C31, SERVO_MOTOR_FREQ, 0);

	pwm_set_duty(dianci_gpio, 0);//先不吸图片
	
	Set_Speed_Wheel(&Motor1_FR, 0);
	Set_Speed_Wheel(&Motor2_FL, 0);
	Set_Speed_Wheel(&Motor3_BL, 0);
	Set_Speed_Wheel(&Motor4_BR, 0);
	
	Status = Status_Stop;
}

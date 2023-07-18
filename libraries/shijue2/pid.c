#include "include.h"
#include "zf_common_headfile.h"
//假设编码器每走50对应路程为1厘米

float Average = 0, Average_FL_BR = 0, Average_FR_BL = 0;

float Change_Position_Temp[2] = {0};

unsigned short int Flag_If_Assigned = 0;
unsigned short int Flag_Equal = 0;

void PID_Cascade(void)
{
//测试区
//***********************************************************************************************************

	
//***********************************************************************************************************
	if (fabs(Motor1_FR.Omega) > 300 || fabs(Motor2_FL.Omega) > 300 || fabs(Motor3_BL.Omega) > 300 || fabs(Motor4_BR.Omega) > 300)
	{
		Flag_000 = 0;
		Status = Status_Stop;
		Initialize();
		printf("Too Fast!\n");
	}
	
	if (Flag_000)//如果打开了开关
	{	
		if (Flag_Acc == 1 && Flag_DeAcc == 0)
		{
			Current_Time += 0.01f;
		}
		else if (Flag_Acc == 0 && Flag_DeAcc == 1)
		{
			Time_DeAcc += 0.01f;
		}
		
		
		if (Status == Status_13 || Status == Status_24)
		{
			if (Flag_Straight_1 == 1 && Flag_Straight_2 == 0 && Flag_Straight_3 ==0 && Flag_Spin_1 == 0 && Flag_Spin_2 == 0 && Flag_Stop == 0)
			{				
				
				Pid_Planning();
				X_Temp = Current_X, Y_Temp = Current_Y;
								
			}
			else if (Flag_Straight_1 == 0 && Flag_Straight_2 == 1 && Flag_Straight_3 == 0 && Flag_Spin_1 == 0 && Flag_Spin_2 == 0 && Flag_Stop == 0)//微调
			{

				PID_Location_All();
				X_Temp = Current_X, Y_Temp = Current_Y;

				//printf("Flag_Straight_2\n");
				//printf("%d, %d, %d, %d, %d, %d, %d\n", Flag_Straight_1, Flag_Straight_2, Flag_Straight_3, Flag_Spin_1, Flag_Spin_2, Flag_Stop, Flag_Run_State);
			}
			else if (Flag_Straight_1 == 0 && Flag_Straight_2 == 0 && Flag_Straight_3 == 1 && Flag_Spin_1 == 0 && Flag_Spin_2 == 0 && Flag_Stop == 0 && Flag_Run_State == 1)//位姿矫正
			{
				PID_Location_All();
				printf("Flag_Straight_3\n");
			}
		}
		else if (Status == Status_Spin)
		{
			if (Flag_Straight_1 == 0 && Flag_Straight_2 == 0 && Flag_Spin_1 == 1 && Flag_Spin_2 == 0 && Flag_Stop == 0)
			{
				
					Pid_Planning();
				
			}
			else if (Flag_Straight_1 == 0 && Flag_Straight_2 == 0 && Flag_Spin_1 == 0 && Flag_Spin_2 == 1 && Flag_Stop == 0)
			{
				if (Status == Status_Spin)
				{
					if (fabs(Angle - Motor1_FR.Target_Angle) < 0.1f && fabs(Motor1_FR.Omega) < 0.3)
					{
						Flag_Spin_2 = 0;
						Flag_Finish = 1;
					}
				}
			}
		}
						
	}
	
	if (Status == Status_Stop || Flag_Spin_2 == 1)
	{
		Motor1_FR.Target_Omega = 0;
		Motor2_FL.Target_Omega = 0;
		Motor3_BL.Target_Omega = 0;
		Motor4_BR.Target_Omega = 0;
		
		Flag_If_Assigned = 1;
	}
	
	if (Status == Status_No_Speed)
	{
		printf("<>");
		Motor1_FR.Target_Omega = Motor1_FR.Omega;
		Motor2_FL.Target_Omega = Motor2_FL.Omega;
		Motor3_BL.Target_Omega = Motor3_BL.Omega;
		Motor4_BR.Target_Omega = Motor4_BR.Omega;
		
		Flag_If_Assigned = 1;
	}
	
	if (0 == Flag_Init_Gyro)
	{
		PID_Gyro();
		PID_All_Speed();
		Set_Speed_All();
		//printf("%f\n", Motor1_FR.Target_Omega);
	}
}

void PID_Acc_Plus(void)//别忘了清零
{

	if (Ay_0 * Trans_AY >= 0 && Ax_0 * Trans_AX > 0)
	{
		Motor1_FR.E3_Acc_Y = Ay_0 - Trans_AY;
		Motor1_FR.E3_Acc_X = Ax_0 - Trans_AX;
		
		Motor1_FR.Error_Acc_Y_temp = PID_Acc_Correction * (Motor1_FR.Kp_Acc * (Motor1_FR.E3_Acc_Y - Motor1_FR.E2_Acc_Y) + Motor1_FR.Ki_Acc * Motor1_FR.E3_Acc_Y + Motor1_FR.Kd_Acc * (Motor1_FR.E3_Acc_Y + 2 * Motor1_FR.E2_Acc_Y + Motor1_FR.E1_Acc_Y));
		Motor1_FR.Error_Acc_X_temp = PID_Acc_Correction * (Motor1_FR.Kp_Acc * (Motor1_FR.E3_Acc_X - Motor1_FR.E2_Acc_X) + Motor1_FR.Ki_Acc * Motor1_FR.E3_Acc_X + Motor1_FR.Kd_Acc * (Motor1_FR.E3_Acc_X + 2 * Motor1_FR.E2_Acc_X + Motor1_FR.E1_Acc_X));

		
		Motor1_FR.E1_Acc_X = Motor1_FR.E2_Acc_X;
		Motor1_FR.E2_Acc_X = Motor1_FR.E3_Acc_X;
		
		Motor1_FR.E1_Acc_Y = Motor1_FR.E2_Acc_Y;
		Motor1_FR.E2_Acc_Y = Motor1_FR.E3_Acc_Y;
	}
	else if (Ay_0 * Trans_AY <= 0 && Ax_0 * Trans_AX <= 0)
	{
		Motor1_FR.E3_Acc_Y = Ay_0 + Trans_AY;
		Motor1_FR.E3_Acc_X = Ax_0 + Trans_AX;
		
		Motor1_FR.Error_Acc_Y_temp = PID_Acc_Correction * (Motor1_FR.Kp_Acc * (Motor1_FR.E3_Acc_Y - Motor1_FR.E2_Acc_Y) + Motor1_FR.Ki_Acc * Motor1_FR.E3_Acc_Y + Motor1_FR.Kd_Acc * (Motor1_FR.E3_Acc_Y + 2 * Motor1_FR.E2_Acc_Y + Motor1_FR.E1_Acc_Y));
		Motor1_FR.Error_Acc_X_temp = PID_Acc_Correction * (Motor1_FR.Kp_Acc * (Motor1_FR.E3_Acc_X - Motor1_FR.E2_Acc_X) + Motor1_FR.Ki_Acc * Motor1_FR.E3_Acc_X + Motor1_FR.Kd_Acc * (Motor1_FR.E3_Acc_X + 2 * Motor1_FR.E2_Acc_X + Motor1_FR.E1_Acc_X));

		
		Motor1_FR.E1_Acc_X = Motor1_FR.E2_Acc_X;
		Motor1_FR.E2_Acc_X = Motor1_FR.E3_Acc_X;
		
		Motor1_FR.E1_Acc_Y = Motor1_FR.E2_Acc_Y;
		Motor1_FR.E2_Acc_Y = Motor1_FR.E3_Acc_Y;

	}

}

void PID_Gyro(void)
{
	if (Flag_If_Assigned == 1)
	{
		Flag_If_Assigned = 0;
		if (Flag_Spin_1 == 0)
		{
			Motor1_FR.E2_Gyro = Angle - Motor1_FR.Target_Angle;
			
			Motor1_FR.Integral_Gyro += Motor1_FR.E2_Gyro;
			
			Motor1_FR.temp = PID_Gyro_Correction * (3 * Motor1_FR.E2_Gyro + 0.01 * Motor1_FR.Integral_Gyro + 0 * (Motor1_FR.E2_Gyro - Motor1_FR.E1_Gyro));
			
			Motor1_FR.Target_Omega += -Motor1_FR.temp;
			Motor2_FL.Target_Omega += Motor1_FR.temp;
			Motor3_BL.Target_Omega += Motor1_FR.temp;
			Motor4_BR.Target_Omega += -Motor1_FR.temp;//转化为每个轮子的角速度
			
			Motor1_FR.E2_Gyro = Motor1_FR.E1_Gyro;
		}
	}
		
}

void PID_Gyro_PWM(void)
{
	if (Flag_Spin_1 == 0)
	{
		Motor1_FR.E2_Gyro = Angle - Motor1_FR.Target_Angle;
		
		Motor1_FR.Integral_Gyro += Motor1_FR.E2_Gyro;
		
		Motor1_FR.temp = PID_Gyro_Correction * (3 * Motor1_FR.E2_Gyro + 0 * Motor1_FR.Integral_Gyro + 0 * (Motor1_FR.E2_Gyro - Motor1_FR.E1_Gyro));
		
		Motor1_FR.PWM += -Motor1_FR.temp;
		Motor2_FL.PWM += Motor1_FR.temp;
		Motor3_BL.PWM += Motor1_FR.temp;
		Motor4_BR.PWM += -Motor1_FR.temp;//转化为每个轮子的角速度
		
		Motor1_FR.E2_Gyro = Motor1_FR.E1_Gyro;
	}
	
//	if (Flag_Spin_2)
//		if (fabs(Motor1_FR.E3_Gyro) < 0.2 && fabs(Motor1_FR.Omega) < 0.4)
//		{
//			Flag_Spin_2 = 0;
//			Flag_Finish = 1;
//		}
}


void PID_Gyro_Plus(void)
{

		Motor1_FR.E3_Gyro = Angle - Motor1_FR.Target_Angle;
		//printf("%f\n", Angle);
		
		//Motor1_FR.Integral_Gyro += Motor1_FR.E2_Gyro;
		
		Motor1_FR.temp = PID_Gyro_Correction * (Motor1_FR.Kp_Gyro * (Motor1_FR.E3_Gyro - Motor1_FR.E2_Gyro) + Motor1_FR.Ki_Gyro * Motor1_FR.E3_Gyro + Motor1_FR.Kd_Gyro * (Motor1_FR.E3_Gyro + 2 * Motor1_FR.E2_Gyro + Motor1_FR.E1_Gyro));
		
	//	if (Status == Status_Spin || Status == Status_Stop)
	//	{
	//		Motor1_FR.Target_Omega -= Motor1_FR.temp;
	//		Motor2_FL.Target_Omega = -Motor1_FR.Target_Omega; 
	//		Motor3_BL.Target_Omega = -Motor1_FR.Target_Omega;
	//		Motor4_BR.Target_Omega = Motor1_FR.Target_Omega;//转化为每个轮子的角速度
	//	}
	//	else if (Status == Status_13 || Status == Status_24)
	//	{
			Motor1_FR.Target_Omega -= Motor1_FR.temp;
			Motor2_FL.Target_Omega += Motor1_FR.temp;
			Motor3_BL.Target_Omega += Motor1_FR.temp;
			Motor4_BR.Target_Omega -= Motor1_FR.temp;//转化为每个轮子的角速度
			//printf("%f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega);
	//	}
		
		Motor1_FR.E1_Gyro = Motor1_FR.E2_Gyro;
		Motor1_FR.E2_Gyro = Motor1_FR.E3_Gyro;
		
	//	
	//	Motor1_FR.E3_Gyro = Angle - Motor1_FR.Target_Angle;
	//		
	//	Motor1_FR.temp = PID_Gyro_Correction * (Motor1_FR.Kp_Gyro * (Motor1_FR.E3_Gyro - Motor1_FR.E2_Gyro) + Motor1_FR.Ki_Gyro * Motor1_FR.E3_Gyro + Motor1_FR.Kd_Gyro * (Motor1_FR.E3_Gyro + 2 * Motor1_FR.E2_Gyro + Motor1_FR.E1_Gyro));
	//	
	//	Motor1_FR.Target_Omega -= Motor1_FR.temp;
	//	Motor2_FL.Target_Omega += Motor1_FR.temp;
	//	Motor3_BL.Target_Omega += Motor1_FR.temp;
	//	Motor4_BR.Target_Omega -= Motor1_FR.temp;//转化为每个轮子的角速度
	//	
	//	Motor1_FR.E1_Gyro = Motor1_FR.E2_Gyro;
	//	Motor1_FR.E2_Gyro = Motor1_FR.E3_Gyro;

	//	if (Status == Status_Spin && Flag_Spin_2 == 1)
	//	{
	//		if (fabs(Motor1_FR.E3_Gyro) < 0.4)
	//		{
	//				Flag_Spin_2 = 0;
	//				Flag_Finish = 1;
	//		}
	//	}
	
}

void PID_All_Speed(void)
{
	
	//printf("%f, %f\n", Motor4_BR.Kp_Speed * Motor4_BR.E2_Speed + Motor4_BR.Kd_Speed * (Motor4_BR.E2_Speed - Motor4_BR.E1_Speed) + Motor4_BR.Ki_Speed * Motor4_BR.Integral_Speed);
	//printf("%f, %f\n", Motor4_BR.Target_Omega - Motor4_BR.Omega, FF_Correction * (Motor4_BR.K * Motor4_BR.Target_Omega + Motor4_BR.B) + PID_Speed_Correction * (Motor4_BR.Kp_Speed * Motor4_BR.E2_Speed + Motor4_BR.Kd_Speed * (Motor4_BR.E2_Speed - Motor4_BR.E1_Speed) + Motor4_BR.Ki_Speed * Motor4_BR.Integral_Speed));
	Pid_Speed(&Motor4_BR);
	//printf("%f, %f, %f, %f, %f, %f\n", Motor4_BR.E2_Speed, Motor4_BR.Target_Omega, Motor4_BR.Omega, Motor2_FL.E2_Speed, Motor2_FL.Target_Omega, Motor2_FL.Omega);
	Pid_Speed(&Motor2_FL);
	Pid_Speed(&Motor1_FR);
	Pid_Speed(&Motor3_BL);
}

void PID_Four_Wheel(void)
{
//	Motor1_FR.Abs_Omega = fabs(Motor1_FR.Omega);
//	Motor2_FL.Abs_Omega = fabs(Motor2_FL.Omega);
//	Motor3_BL.Abs_Omega = fabs(Motor3_BL.Omega);
//	Motor4_BR.Abs_Omega = fabs(Motor4_BR.Omega);
	
		if (Status == Status_13)
		{
//			if (Motor2_FL.Omega >= 0)
//				Average_FL_BR = (Motor2_FL.Abs_Omega + Motor4_BR.Abs_Omega) / 2.0f;
//			else
//				Average_FL_BR = -(Motor2_FL.Abs_Omega + Motor4_BR.Abs_Omega) / 2.0f;
			Average_FL_BR = (Motor2_FL.Omega + Motor4_BR.Omega) / 2;
			Average_FR_BL = rate * Average_FL_BR;
			
			PID_Average(&Motor1_FR, (&Average_FR_BL));
			PID_Average(&Motor2_FL, (&Average_FL_BR));
			PID_Average(&Motor3_BL, (&Average_FR_BL));
			PID_Average(&Motor4_BR, (&Average_FL_BR));

		}
		else if (Status == Status_24)
		{
//			if (Motor1_FR.Omega >= 0)
//				Average_FR_BL = (Motor1_FR.Abs_Omega + Motor3_BL.Abs_Omega) / 2.0f;
//			else
//				Average_FR_BL = -(Motor1_FR.Abs_Omega + Motor3_BL.Abs_Omega) / 2.0f;
			Average_FR_BL = (Motor1_FR.Omega + Motor3_BL.Omega) / 2;
			Average_FL_BR = rate * Average_FR_BL;
			
			PID_Average(&Motor1_FR, (&Average_FR_BL));
			PID_Average(&Motor2_FL, (&Average_FL_BR));
			PID_Average(&Motor3_BL, (&Average_FR_BL));
			PID_Average(&Motor4_BR, (&Average_FL_BR));

		}
		else if (Status == Status_Spin || Status == Status_Stop)
		{
			if (Motor1_FR.Abs_Omega > 0)
				Average_FL_BR = (fabs(Motor2_FL.Omega) + fabs(Motor4_BR.Omega) + fabs(Motor1_FR.Omega) + fabs(Motor3_BL.Omega)) / 4.0f;
			else
				Average_FL_BR = -(fabs(Motor2_FL.Omega) + fabs(Motor4_BR.Omega) + fabs(Motor1_FR.Omega) + fabs(Motor3_BL.Omega)) / 4.0f;
			
			PID_Average(&Motor1_FR, (&Average_FL_BR));
			Motor2_FL.Target_Omega = -Motor1_FR.Target_Omega;
			Motor3_BL.Target_Omega = -Motor1_FR.Target_Omega;
			Motor4_BR.Target_Omega = Motor1_FR.Target_Omega;
			
			Flag_If_Assigned = 1;
		}
}

void PID_Average(Motor_DataTypedef* Motor, float* num)//可能会出现问题：当轮子换转动方向时
{
	Motor->E3_Average = (*num) - Motor->Omega;
	Motor->Target_Omega += PID_Four_Correction * (Motor->Kp_Average * (Motor->E3_Average - Motor->E2_Average) + Motor->Ki_Average * (Motor->E3_Average) + Motor->Kd_Average * (Motor->E3_Average - 2 * Motor->E2_Average + Motor->E1_Average));
	Motor->E1_Average = Motor->E2_Average;
	Motor->E2_Average = Motor->E3_Average;
}

	


void Pid_Planning(void)//自己想的速度规划版本
{
#if 1
	if (Status == Status_13)//以FL为基础
	{
		if (Flag_Acc)//1. 加速阶段
		{
			if (fabs(Motor2_FL.Target_Omega) >= fabs(Motor2_FL.Needed_Omega_Max))//如果速度超过了
			{
				Flag_Acc = 0;
				Flag_DeAcc = 0;
				//Flag_Finish = 1;//测试用
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
				
				Flag_If_Assigned = 1;
			}
			else if (Flag_Acc)//这才是真正的加速的阶段
			{
				Motor1_FR.Target_Omega = 0.03f * Motor1_FR.Needed_Omega_Max + Motor1_FR.Needed_Omega_Acc * Current_Time;
				Motor2_FL.Target_Omega = 0.03f * Motor2_FL.Needed_Omega_Max + Motor2_FL.Needed_Omega_Acc * Current_Time;
				Motor3_BL.Target_Omega = 0.03f * Motor3_BL.Needed_Omega_Max + Motor3_BL.Needed_Omega_Acc * Current_Time;
				Motor4_BR.Target_Omega = 0.03f * Motor4_BR.Needed_Omega_Max + Motor4_BR.Needed_Omega_Acc * Current_Time;
				
				Flag_If_Assigned = 1;
			}		
		}
		else if (fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < fabs(Motor2_FL.S1))//2. 减速阶段
		{
			Flag_Acc = 0;
			Flag_DeAcc = 1;
			//printf("Motor2_FL: %f, %f\n", Motor2_FL.Target_Omega, Motor2_FL.Needed_Omega_Acc);
			if (Motor2_FL.Target_Omega * Motor2_FL.Needed_Omega_Acc < 0)
			{
				Motor1_FR.Target_Omega = 0;
				Motor2_FL.Target_Omega = 0;
				Motor3_BL.Target_Omega = 0;
				Motor4_BR.Target_Omega = 0;
				
				Flag_If_Assigned = 1;
				
				Flag_DeAcc = 0;
				Flag_Acc = 0;

				if (Flag_Straight_1 == 1)
				{
					Flag_Straight_1 = 0;
					Flag_Straight_2 = 1;
				}
				else if (Flag_Spin_1 == 1)
				{
					Flag_Spin_1 = 0;
					Flag_Spin_2 = 1;
				}
			}
			else if (Flag_DeAcc)
			{
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max - Motor1_FR.Needed_Omega_Acc * Time_DeAcc;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max - Motor2_FL.Needed_Omega_Acc * Time_DeAcc;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max - Motor3_BL.Needed_Omega_Acc * Time_DeAcc;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max - Motor4_BR.Needed_Omega_Acc * Time_DeAcc;
				
				Flag_If_Assigned = 1;
			}
		}
		else if (Flag_Acc == 0 && Flag_DeAcc == 0)//3. 匀速阶段
		{
			Flag_Acc = 0;
			Flag_DeAcc = 0;
			Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
			Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
			Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
			Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
			
			Flag_If_Assigned = 1;
		}
	}
	else if (Status == Status_24)
	{
		if (Flag_Acc)//1. 加速阶段
		{
			if (fabs(Motor1_FR.Target_Omega) >= fabs(Motor1_FR.Needed_Omega_Max))//如果速度超过了
			{
				Flag_Acc = 0;
				Flag_DeAcc = 0;
				//Flag_Finish = 1;
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
				
				Flag_If_Assigned = 1;
			}
			else if (Flag_Acc)//这才是真正的加速的阶段
			{
				Motor1_FR.Target_Omega = 0.03f * Motor1_FR.Needed_Omega_Max + Motor1_FR.Needed_Omega_Acc * Current_Time;
				Motor2_FL.Target_Omega = 0.03f * Motor2_FL.Needed_Omega_Max + Motor2_FL.Needed_Omega_Acc * Current_Time;
				Motor3_BL.Target_Omega = 0.03f * Motor3_BL.Needed_Omega_Max + Motor3_BL.Needed_Omega_Acc * Current_Time;
				Motor4_BR.Target_Omega = 0.03f * Motor4_BR.Needed_Omega_Max + Motor4_BR.Needed_Omega_Acc * Current_Time;
				
				Flag_If_Assigned = 1;
			}		
		}
		else if (fabs(Motor1_FR.Needed_Theta - Motor1_FR.Theta) < fabs(Motor1_FR.S1))//2. 减速阶段
		{
			Flag_Acc = 0;
			Flag_DeAcc = 1;
			//printf("Motor1_FR: %f, %f\n", Motor1_FR.Target_Omega, Motor1_FR.Needed_Omega_Acc);
			if (Motor1_FR.Target_Omega * Motor1_FR.Needed_Omega_Acc < 0)
			{
				Motor1_FR.Target_Omega = 0;
				Motor2_FL.Target_Omega = 0;
				Motor3_BL.Target_Omega = 0;
				Motor4_BR.Target_Omega = 0;
				
				Flag_If_Assigned = 1;
				
				Flag_DeAcc = 0;
				Flag_Acc = 0;

				if (Flag_Straight_1 == 1)
				{
					Flag_Straight_1 = 0;
					Flag_Straight_2 = 1;
				}
				else if (Flag_Spin_1 == 1)
				{
					Flag_Spin_1 = 0;
					Flag_Spin_2 = 1;
				}
			}
			else if (Flag_DeAcc)
			{
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max - Motor1_FR.Needed_Omega_Acc * Time_DeAcc;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max - Motor2_FL.Needed_Omega_Acc * Time_DeAcc;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max - Motor3_BL.Needed_Omega_Acc * Time_DeAcc;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max - Motor4_BR.Needed_Omega_Acc * Time_DeAcc;
				
				Flag_If_Assigned = 1;
			}
		}
		else if (Flag_Acc == 0 && Flag_DeAcc == 0)//3. 匀速阶段
		{
			Flag_Acc = 0;
			Flag_DeAcc = 0;
			Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
			Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
			Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
			Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
			
			Flag_If_Assigned = 1;
		}
	}
	else if (Status == Status_Spin)
	{
#if 0
		if (Flag_Acc)//1. 
		{
			if (fabs(Motor1_FR.Target_Omega) >= fabs(Motor1_FR.Needed_Omega_Max))//如果速度超过了
			{
				Flag_Acc = 0;
				Flag_DeAcc = 0;
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
			}
			else if (Flag_Acc)//这才是真正的加速的阶段
			{
				Motor1_FR.Target_Omega = 0.3f * Motor1_FR.Needed_Omega_Max + Motor1_FR.Needed_Omega_Acc * Current_Time;
				Motor2_FL.Target_Omega = 0.3f * Motor2_FL.Needed_Omega_Max + Motor2_FL.Needed_Omega_Acc * Current_Time;
				Motor3_BL.Target_Omega = 0.3f * Motor3_BL.Needed_Omega_Max + Motor3_BL.Needed_Omega_Acc * Current_Time;
				Motor4_BR.Target_Omega = 0.3f * Motor4_BR.Needed_Omega_Max + Motor4_BR.Needed_Omega_Acc * Current_Time;
			}		
		}
		else if (fabs(Motor1_FR.Needed_Theta - Motor1_FR.Theta) < fabs(Motor1_FR.S1))//2. 
		{
			Flag_Acc = 0;
			Flag_DeAcc = 1;
			if (Motor1_FR.Omega * Motor1_FR.Needed_Omega_Acc < 0)
			{
				Motor1_FR.Target_Omega = 0;
				Motor2_FL.Target_Omega = 0;
				Motor3_BL.Target_Omega = 0;
				Motor4_BR.Target_Omega = 0;
				Flag_DeAcc = 0;
				Flag_Acc = 0;
				if (Flag_Straight_1 == 1)
				{
					Flag_Straight_1 = 0;
					Flag_Straight_2 = 1;
				}
				else if (Flag_Spin_1 == 1)
				{
					Flag_Spin_1 = 0;
					Flag_Spin_2 = 1;
				}		
			}
			else if (Flag_DeAcc)
			{
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max - Motor1_FR.Needed_Omega_Acc * Time_DeAcc;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max - Motor2_FL.Needed_Omega_Acc * Time_DeAcc;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max - Motor3_BL.Needed_Omega_Acc * Time_DeAcc;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max - Motor4_BR.Needed_Omega_Acc * Time_DeAcc;
			}

		}
		else if (Flag_Acc == 0 && Flag_DeAcc == 0)//3. 匀速阶段
		{
			Flag_Acc = 0;
			Flag_DeAcc = 0;
			Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
			Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
			Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
			Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
		}
#endif

#if 1
		if (Motor1_FR.Target_Omega > 0)
		{
			if (Angle > Motor1_FR.Target_Angle)
			{
				Flag_Spin_1 = 0;
				Flag_Spin_2 = 1;
			}			
		}
		else if (Motor1_FR.Target_Omega < 0)
		{
			if (Angle < Motor1_FR.Target_Angle)
			{
				Flag_Spin_1 = 0;
				Flag_Spin_2 = 1;
			}
		}
#endif

#if 0
		if (Flag_Acc)//1. 加速阶段
		{
			
			if (fabs(Motor1_FR.Target_Omega) >= fabs(Motor1_FR.Needed_Omega_Max))//如果速度超过了
			{
				Flag_Acc = 0;
				Flag_DeAcc = 1;
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max;
				//printf("??");
			}
			else if (Flag_Acc)//这才是真正的加速的阶段
			{
				//printf("??");
				Motor1_FR.Target_Omega = 0.03f * Motor1_FR.Needed_Omega_Max + Motor1_FR.Needed_Omega_Acc * Current_Time;
				Motor2_FL.Target_Omega = 0.03f * Motor2_FL.Needed_Omega_Max + Motor2_FL.Needed_Omega_Acc * Current_Time;
				Motor3_BL.Target_Omega = 0.03f * Motor3_BL.Needed_Omega_Max + Motor3_BL.Needed_Omega_Acc * Current_Time;
				Motor4_BR.Target_Omega = 0.03f * Motor4_BR.Needed_Omega_Max + Motor4_BR.Needed_Omega_Acc * Current_Time;
			}		
		}
		else if (Flag_DeAcc == 1)//2. 减速阶段
		{
			//printf("..");
			//printf("%f\n", Motor1_FR.Target_Omega);
			if (Motor1_FR.Target_Omega * Motor1_FR.Needed_Omega_Acc < 0)
			{
//				Motor1_FR.Target_Omega = 0;
//				Motor2_FL.Target_Omega = 0;
//				Motor3_BL.Target_Omega = 0;
//				Motor4_BR.Target_Omega = 0;
				
				Flag_DeAcc = 0;
				Flag_Acc = 0;


				Flag_Spin_1 = 0;
				Flag_Finish = 1;
				Status = Status_Stop;
				
			}
			else
			{
				Motor1_FR.Target_Omega = Motor1_FR.Needed_Omega_Max - Motor1_FR.Needed_Omega_Acc * Time_DeAcc;
				Motor2_FL.Target_Omega = Motor2_FL.Needed_Omega_Max - Motor2_FL.Needed_Omega_Acc * Time_DeAcc;
				Motor3_BL.Target_Omega = Motor3_BL.Needed_Omega_Max - Motor3_BL.Needed_Omega_Acc * Time_DeAcc;
				Motor4_BR.Target_Omega = Motor4_BR.Needed_Omega_Max - Motor4_BR.Needed_Omega_Acc * Time_DeAcc;
			}
		}
		#endif
	}
	
#endif
}

void PID_Location_All(void)
{
		
	if (Flag_Straight_2 == 1 && Flag_Run_Normal == 1)
	{
		Flag_Straight_2 = 0;
		if (Flag_Run_State)			
		{
			Flag_Straight_3 = 1;
			Flag_Correct = 1;
			printf("I will in Flag_Straight_3!\n");
		}
		else
		{
			Flag_Finish = 1;
		}//07.15测试，直接进位姿矫正或者结束
	}
	else
	{

		
		
		if (Flag_Straight_2 == 1 && Flag_Straight_3 == 0)
		{

					
				//printf("%d, %d\n", fabs(Angle - Motor1_FR.Target_Angle) < 0.8 , fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 1);
				
				if (   (Status == Status_13 && (fabs(Angle - Motor1_FR.Target_Angle) < 0.2 && fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 0.2))    ||   (Status == Status_24 && (fabs(Angle - Motor1_FR.Target_Angle) < 0.2 && fabs(Motor1_FR.Needed_Theta - Motor1_FR.Theta) < 0.2))  )
				{
					Flag_Straight_2 = 0;
					if (Flag_Run_State)//
					{
						Flag_Straight_3 = 1;
						Flag_Correct = 1;
						printf("I will in Flag_Straight_3!\n");
					}
					else
					{
						Flag_Finish = 1;
					}
					//Clear_To_Correct();	
				}
				if (Status == Status_13)
					Motor1_FR.Theta = rate * Motor2_FL.Theta;
				else if (Status == Status_24)
					Motor2_FL.Theta = rate * Motor1_FR.Theta;
				Motor3_BL.Theta = Motor1_FR.Theta;
				Motor4_BR.Theta = Motor2_FL.Theta;
				


			//printf("%f, %f, %f, %f, %d, %d ,%d, %d, %f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega, Motor1_FR.PWM, Motor2_FL.PWM, Motor3_BL.PWM, Motor4_BR.PWM, Motor1_FR.Omega, Motor2_FL.Omega,Motor3_BL.Omega,Motor4_BR.Omega);
			//}
		}
		else if (Flag_Straight_2 == 0 && Flag_Straight_3 == 1)
		{
			if (Flag_Correct)
			{//printf("nnum %f, %f\n", nnum1, nnum2);
					
					Flag_Correct = 0;
					Clear_To_Correct();
				if (1)
				{
					printf("有图片");					
					Set_Location_Correction(change_position[0], change_position[1]);
					Flag_If_Received_Picture_Dot = 0;
				}
				else
				{
					printf("没图片好像");
					Flag_Straight_3 = 0;
					Flag_Correct = 1;
					Flag_Finish = 1;
				}		
			}
			
			if (Change_Position_Temp[0] == change_position[0] && Change_Position_Temp[1] == change_position[1])
			{
				Flag_Equal++;
			}
			Change_Position_Temp[0] = change_position[0];
			Change_Position_Temp[1] = change_position[1];

			if (Status == Status_13)
				Motor1_FR.Theta = rate * Motor2_FL.Theta;
			else if (Status == Status_24)
				Motor2_FL.Theta = rate * Motor1_FR.Theta;
			Motor3_BL.Theta = Motor1_FR.Theta;
			Motor4_BR.Theta = Motor2_FL.Theta;
			
			
			if (((fabs(change_position[0]) < 0.015f) && (fabs(change_position[1]) < 0.015f))  ||  Flag_Equal >= 50)
			{
				printf("[[");
				Flag_Straight_3 = 0;
				Flag_Correct = 1;
				Flag_Finish = 1;
				Flag_Equal = 0;
			}
			
			
			
			if (Status == Status_13)
			{
				if (fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 0.3)
				{
					Flag_Correct = 1;		
				}
			}
			else if (Status == Status_24)
			{
				if (fabs(Motor1_FR.Needed_Theta - Motor1_FR.Theta) < 0.3)
				{
					Flag_Correct = 1;		
				}
			}
		
		}

		Pid_Location(&Motor1_FR);
		Pid_Location(&Motor2_FL);
		Pid_Location(&Motor3_BL);
		Pid_Location(&Motor4_BR);
		
		
		
	}
}

void Pid_Location_Plus_All(void)
{
	Motor1_FR.E3_Location = Motor1_FR.Needed_Theta - Motor1_FR.Theta;
	Motor2_FL.E3_Location = Motor2_FL.Needed_Theta - Motor2_FL.Theta;
	//>0
	Motor1_FR.Error_Theta_temp = PID_Location_Correction * (Motor1_FR.Kp_Location * (Motor1_FR.E3_Location - Motor1_FR.E2_Location) + Motor1_FR.Ki_Location * (Motor1_FR.E3_Location) + Motor1_FR.Kd_Location * (Motor1_FR.E3_Location - 2 * Motor1_FR.E2_Location + Motor1_FR.E1_Location));
	Motor2_FL.Error_Theta_temp = PID_Location_Correction * (Motor2_FL.Kp_Location * (Motor2_FL.E3_Location - Motor2_FL.E2_Location) + Motor2_FL.Ki_Location * (Motor2_FL.E3_Location) + Motor2_FL.Kd_Location * (Motor2_FL.E3_Location - 2 * Motor2_FL.E2_Location + Motor2_FL.E1_Location));
	//<0
	Motor1_FR.Target_Omega += Motor1_FR.Error_Theta_temp;
	Motor2_FL.Target_Omega += Motor2_FL.Error_Theta_temp;
	Motor3_BL.Target_Omega += Motor1_FR.Error_Theta_temp;
	Motor4_BR.Target_Omega += Motor2_FL.Error_Theta_temp;
	
	Motor1_FR.E1_Location = Motor1_FR.E2_Location;
	Motor2_FL.E1_Location = Motor2_FL.E2_Location;
	
	Motor1_FR.E2_Location = Motor1_FR.E3_Location;
	Motor2_FL.E2_Location = Motor2_FL.E3_Location;
}

void Pid_Location_Plus(Motor_DataTypedef* Motor)//新版，增量式
{
	Motor->E3_Location = Motor->Needed_Theta - Motor->Theta;
//	if (Motor->E2_Location >= -Location_DeadZone && Motor->E2_Location <= Location_DeadZone)//闭环死区
//	{
//		Motor->E1_Location = 0;
//		Motor->E2_Location = 0;
//		Motor->E3_Location = 0;
//	}
	Motor->Error_Theta_temp = PID_Location_Correction * (Motor->Kp_Location * (Motor->E3_Location - Motor->E2_Location) + Motor->Ki_Location * (Motor->E3_Location) + Motor->Kd_Location * (Motor->E3_Location - 2 * Motor->E2_Location + Motor->E1_Location));
	//Motor->Target_Omega += Motor->Error_Theta_temp;
	Motor->E1_Location = Motor->E2_Location;
	Motor->E2_Location = Motor->E3_Location;

	//				Motor1_FR.E3_Gyro = Angle - Motor1_FR.Target_Angle;
//	
//	//Motor1_FR.Integral_Gyro += Motor1_FR.E2_Gyro;
//	
//	Motor1_FR.temp = PID_Gyro_Correction * (Motor1_FR.Kp_Gyro * (Motor1_FR.E3_Gyro - Motor1_FR.E2_Gyro) + Motor1_FR.Ki_Gyro * Motor1_FR.E3_Gyro + Motor1_FR.Kd_Gyro * (Motor1_FR.E3_Gyro + 2 * Motor1_FR.E2_Gyro + Motor1_FR.E1_Gyro));
//	
//	if (Status == Status_Spin || Status == Status_Stop)
//	{
//		Motor1_FR.Target_Omega -= Motor1_FR.temp;
//		Motor2_FL.Target_Omega = -Motor1_FR.Target_Omega;
//		Motor3_BL.Target_Omega = -Motor1_FR.Target_Omega;
//		Motor4_BR.Target_Omega = Motor1_FR.Target_Omega;//转化为每个轮子的角速度
//	}
//	else if (Status == Status_13 || Status == Status_24)
//	{
//		Motor1_FR.Target_Omega -= Motor1_FR.temp;
//		Motor2_FL.Target_Omega += Motor1_FR.temp;
//		Motor3_BL.Target_Omega += Motor1_FR.temp;
//		Motor4_BR.Target_Omega -= Motor1_FR.temp;//转化为每个轮子的角速度
//	}
//	
//	Motor1_FR.E1_Gyro = Motor1_FR.E2_Gyro;
//	Motor1_FR.E2_Gyro = Motor1_FR.E3_Gyro;
}

void Pid_Location(Motor_DataTypedef* Motor)
{
#if 1 //旧版，位置式
	if (Status == Status_Spin);//调试用
	else
		Motor->E2_Location = (Motor->Needed_Theta - Motor->Theta);
	if (Motor->E2_Location >= -Location_DeadZone && Motor->E2_Location <= Location_DeadZone)//闭环死区
	{
		Motor->E1_Location = 0;
		Motor->E2_Location = 0;
		Motor->Integral_Location = 0;
	}
//	
//	if(Motor->E2_Location > -Max_Err_Location && Motor->E2_Location < Max_Err_Location)//积分分离
//  {
		Motor->Integral_Location += Motor->E2_Location;  
        /*积分范围限定，防止积分饱和*/
		LimitMax(Motor->Integral_Location, Max_Integral_Location);
//  }
//	
	Motor->Target_Omega = (Motor->Kp_Location * Motor->E2_Location + Motor->Kd_Location * (Motor->E2_Location - Motor->E1_Location) + Motor->Ki_Location * Motor->Integral_Location);
	Flag_If_Assigned = 1;
	LimitMax(Motor->Target_Omega, Max_Omega);
	Motor->E1_Location = Motor->E2_Location;
#endif
	
}

void Pid_Speed(Motor_DataTypedef* Motor)
{
	
//	if (Motor->temp <= 100)
//	{
//		Motor->temp++;
//		Motor->PWM = Motor->K * Motor->Target_Omega + Motor->B;

//	}
//	else if (Motor->temp > 100)
//	{
	Motor->E2_Speed = Motor->Target_Omega - Motor->Omega;
//	if (Motor->E2_Speed >= -Speed_DeadZone && Motor->E2_Speed <= Speed_DeadZone)//闭环死区
//	{
////		Motor->E1_Speed = 0;
////		Motor->E2_Speed = 0;
//		Motor->Integral_Speed /= 2;
//	}
	
//	if(Motor->E2_Speed > -Max_Err_Speed && Motor->E2_Speed < Max_Err_Speed)//积分分离
//  {
//	if (fabs(Motor->E2_Speed) < 10 && fabs(Motor->E2_Speed) > 1)
//		Motor->Integral_Speed += Motor->E2_Speed; 
//	else if (fabs(Motor->E2_Speed) < 5)	
//		Motor->Integral_Speed += 100 / Motor->E2_Speed;
//	else
	if (Flag_DeAcc)
		Motor->Integral_Speed += 10 * Motor->E2_Speed; 
	else
		Motor->Integral_Speed += Motor->E2_Speed; 
				/*积分范围限定，防止积分饱和*/
		LimitMax(Motor->Integral_Speed, Max_Integral_Speed);
//  }
	if (Motor->Target_Omega > 0) 
		Motor->PWM = FF_Correction * (Motor->K * Motor->Target_Omega + Motor->B) + PID_Speed_Correction * (Motor->Kp_Speed * Motor->E2_Speed + Motor->Kd_Speed * (Motor->E2_Speed - Motor->E1_Speed) + Motor->Ki_Speed * Motor->Integral_Speed);
	else if (Motor->Target_Omega < 0)
		Motor->PWM = FF_Correction * (Motor->K * Motor->Target_Omega - Motor->B) + PID_Speed_Correction * (Motor->Kp_Speed * Motor->E2_Speed + Motor->Kd_Speed * (Motor->E2_Speed - Motor->E1_Speed) + Motor->Ki_Speed * Motor->Integral_Speed);

//	Motor->PWM = Motor->K * Motor->Target_Omega + Motor->B;
	//?
	
	Motor->E1_Speed = Motor->E2_Speed;
//	}
	LimitMax(Motor->PWM, Max_PWM);
}


//void first_pid(Motor_DataTypedef* Motor, short int x0)
//{
//	Motor->E3 =x0- Motor->Encoder;
//	Motor->V += (Kp * Motor->E3 + Kd * (Motor->E2 - Motor->E3) + Ki * (Motor->E3 + Motor->E2 + Motor->E1));
//	Set_Speed_Wheel(Motor, 600+Motor->V);
//	Motor->E1 = Motor->E2;
//	Motor->E2 = Motor->E3;
//}
//int pid_pid(Motor_DataTypedef* Motor,short int ordered_meter,int now_speed,int changeing_speed)
//{
//	int x;
//	Motor->Ex3=ordered_meter-already_meter;//ordered_meter为250,already_meter为个位数
//  Motor->x=(Kxp * Motor->Ex3 + Kxd * (Motor->Ex2 - Motor->Ex3) + Kxi * (Motor->Ex3 + Motor->Ex2 + Motor->Ex1));	//期望的速度，在500-1500中间浮动
//	LimitMax(Motor->x,1500);
//	Motor->Ex1 = Motor->Ex2;
//	Motor->Ex2 = Motor->Ex3;
//	
//	Motor->E3 =Motor->x-now_speed;
//	Motor->V = (Kp * Motor->E3 + Kd * (Motor->E2 - Motor->E3) + Ki * (Motor->E3 + Motor->E2 + Motor->E1));
//	LimitMax(Motor->V,300);
//	x=changeing_speed;
//	LimitMax(x,500);
//  float mid=(float)(Motor->Encoder-x);
//	if(mid>=0){}
//		else{
//		mid=-mid;
//		}
//  while(mid>=10){
//  first_pid(Motor,x);
//	}
//	Motor->E1 = Motor->E2;
//	Motor->E2 = Motor->E3;
//	return Motor->V;
//}

/*************************************************************/

//void PID_Init(pid_type_def*pid,int mode,const float PID[3],float max_out,float imax_out)
//{
//	pid->mode=mode;
//	
//	pid->kp=PID[0];
//  pid->ki=PID[1];
//	pid->kd=PID[2];
//	
//	pid->max_out=max_out;
//	pid->imax_out=imax_out;
//  
//	pid->dbuf[0]=pid->dbuf[1]=pid->dbuf[2]=0.0f;
//	pid->error[0] = pid->error[1] = pid->error[2] = pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
//}
//float PID_calculate(pid_type_def*pid,float now,float set)
//{ 
//	if(pid==NULL)
//	{
//		return 0.0f;
//	}
//	pid->error[2]=pid->error[1];
//	pid->error[1]=pid->error[0];

//  pid->set=set;
//	pid->now=now;
//	
//	pid->error[0]=set-now;
//	
//	
//	
//	if(pid->mode==PID_POSITION)
//	{
//		pid->pout=pid->kp*pid->error[0];
//		
//		
//		pid->iout+=pid->ki*pid->error[0];
//		
//		pid->dbuf[2] = pid->dbuf[1];
//    pid->dbuf[1] = pid->dbuf[0];
//		
//		pid->dbuf[0]=pid->error[0]-pid->error[1];
//		pid->dout = pid->kd * pid->dbuf[0];
//		
//		//对积分项进行限幅
//    LimitMax(pid->iout, pid->imax_out);
//		
//		//叠加三个输出到总输出
//    pid->out = pid->pout + pid->iout + pid->dout;
//		
//		
//    //对总输出进行限幅
//    LimitMax(pid->out, pid->max_out);
//	}else if(pid->mode==PID_enhance)
//	{
//	  pid->pout=pid->kp*(pid->error[0]-pid->error[1]);
//		pid->iout=pid->ki*pid->error[0];
//		
//		pid->dbuf[2]=pid->dbuf[1];
//		pid->dbuf[1]=pid->dbuf[0];
//		pid->dbuf[0]=pid->dbuf[0]-2.0f*pid->dbuf[1]+pid->dbuf[2];
//		
//		pid->dout=pid->kd*pid->dbuf[0];
//		
//		pid->out+=pid->pout+pid->dout;
//		LimitMax(pid->out,pid->max_out);
//	
//	}
//return pid->out;
//}

//void PID_clear(pid_type_def *pid)
//{
//    if (pid == NULL)
//    {
//        return;
//    }

//    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;

//    pid->dbuf[0] = pid->dbuf[1] = pid->dbuf[2] = 0.0f;

//    pid->out = pid->pout = pid->pout = pid->pout = 0.0f;

//    pid->now = pid->set = 0.0f;
//}

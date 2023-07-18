#include "include.h"
unsigned short int print = 0;
void Pit_Handler_PID(void)
{		
#if 1
//		if (print < 50)
//		{
//			printf("456");
//			print++;
//			Gyro_Initialize();
//		}
	num_of_time++;
	
	
	
	//Gyro_Test();
	Gyro_Cal();
	//Gyro_Cal_2();
	
	
	
	if (2 == num_of_time)
	{
		num_of_time = 0;
		
		
		//printf("%f\n", Motor2_FL.Omega);
		
		Get_Encoder();
		Calculate();
		Clear_Encoder();
		
		//printf("%f\n", Motor2_FL.Needed_Theta);
		PID_Cascade();
		//printf("%f, %f\n", B_Hat, A_Hat);
		//printf("%f, %f\n", Angle, temp_angle);
		//Get_Location();
		//printf("%f, %f\n", Current_X, Current_Y);
		//printf("%f ,%f, %f, %f\n", Motor1_FR.Theta, Motor2_FL.Theta, Motor3_BL.Theta, Motor4_BR.Theta);
	}
		
		
//			Matrix();
//			if (fabs(Trans_X) < 0.005)
//				Trans_X = 0;
//			if (fabs(Trans_Y) < 0.005)
//				Trans_Y = 0;
//			Trans_AX = Trans_X / (Trans_X * Trans_X + Trans_Y * Trans_Y);
//			Trans_AY = Trans_Y / (Trans_X * Trans_X + Trans_Y * Trans_Y);
			//if (Flag_000)
				//printf("%f, %d, %d, %d, %d, %d, %d\n", Angle, imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z);
	//printf("%f, %f, %f, %f, %f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega, Motor1_FR.Omega, Motor2_FL.Omega,Motor3_BL.Omega,Motor4_BR.Omega);

		
#endif
#if 0
		num_of_time++;
		print++;
	
		//PID_Cascade();
	
		if (num_of_time == 2)
		{
//			Get_Encoder();
//			Clear_Encoder();
//			Calculate_Pulse();//考虑加在后面

//			Calculate();
		}
		else if (num_of_time >= 10)
		{
			
			num_of_time = 1;
			
			
//			PWM_Test();
			
	
//			PID_Test();
// 			printf("%f, %f, %f, %f, %d, %d, %d, %d, %f, %d, %d, %d, %f\n", Motor1_FR.Omega,\
			Motor2_FL.Omega,Motor3_BL.Omega, Motor4_BR.Omega,Motor1_FR.Pulse_Count,\
			Motor2_FL.Pulse_Count,Motor3_BL.Pulse_Count,Motor4_BR.Pulse_Count,\
			Motor2_FL.Target_Omega,Motor2_FL.E2_Speed,Motor2_FL.Integral_Speed,Motor2_FL.Needed_Pulse_Count,\
			Motor1_FR.Acc);	
//			printf("%d, %d, %d, %d, %d\n", sum[0], sum[1], sum[2], sum[3], Motor1_FR.PWM);
//			printf("%f, %d, %d\n", Motor2_FL.Omega, Motor2_FL.Encoder, Motor2_FL.Pulse_Count);
//			printf("%f\n", Motor3_BL.Omega);
//			printf("%f, %f, %f, %f, %f, %f, %f, %f\n", Motor1_FR.Omega, Motor2_FL.Omega, Motor3_BL.Omega, Motor4_BR.Omega, Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega);
//			printf("%d, %d\n", Motor2_FL.Pulse_Count, Motor1_FR.Pulse_Count);
//			printf("%f, %f, %f, %f, %f, %f, %f\n",Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega,\
			Motor2_FL.S1, Motor2_FL.Needed_Theta - Motor2_FL.Theta, Motor2_FL.Theta);
//			printf("%f, %f, %f, %f\n", Motor1_FR.Needed_Omega_Acc, Motor2_FL.Needed_Omega_Acc, Motor1_FR.Needed_Omega_Max, Motor2_FL.Needed_Omega_Max);
//		printf("%f\n", Angle);
//			printf("%f, %f, %d\n", Motor2_FL.E2_Speed, Motor2_FL.Integral_Speed, Motor2_FL.PWM);
//			printf("%f, %f, %f, %f\n",Motor1_FR.Theta, Motor2_FL.Theta, Motor3_BL.Theta, Motor4_BR.Theta);
//			printf("%f\n", z_degree);
			//printf("%f, %f, %f, %f, %f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega, Motor1_FR.Omega, Motor2_FL.Omega,Motor3_BL.Omega,Motor4_BR.Omega);
//			printf("%f, %f, %f, %f\n", Motor1_FR.Needed_Theta, Motor2_FL.Needed_Theta, Motor3_BL.Needed_Theta, Motor4_BR.Needed_Theta);
			//printf("%f, %f, %f, %f, %f, %f, %d, %d\n", Motor1_FR.Omega, Motor2_FL.Omega, Motor3_BL.Omega, Motor4_BR.Omega, Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor1_FR.PWM, Motor2_FL.PWM);
			//printf("%d, %d, %d, %f, %f, %f\n", fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 0.2, fabs(Motor2_FL.Omega) < 0.01, fabs(Angle) < 0.1,Motor2_FL.Needed_Theta - Motor2_FL.Theta,Motor2_FL.Omega, Angle);
			//printf("%d, %d, %d\n",fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 0.2, Motor2_FL.Omega == 0, fabs(Motor2_FL.Needed_Theta - Motor2_FL.Theta) < 0.2 && Motor2_FL.Omega == 0);
			//printf("%f, %f\n", Angle, Motor1_FR.E3_Gyro);
			//printf("%f, %f\n", Motor1_FR.Error_Acc_X_temp, Motor1_FR.Error_Acc_Y_temp);
		}
		else if (num_of_time == 4)
		{
			Gyro_Cal();
		}			
		else if (num_of_time == 6)
		{
//			Matrix();
//			if (fabs(Trans_X) < 0.005)
//				Trans_X = 0;
//			if (fabs(Trans_Y) < 0.005)
//				Trans_Y = 0;
//			Trans_AX = Trans_X / (Trans_X * Trans_X + Trans_Y * Trans_Y);
//			Trans_AY = Trans_Y / (Trans_X * Trans_X + Trans_Y * Trans_Y);
		}
		
		
		
		if (print >= 37)
		{
			//printf("%f, %f, %f, %f, %d\n", Motor4_BR.E1_Speed, Motor4_BR.E2_Speed, Motor4_BR.Target_Omega, Motor4_BR.Omega, Motor4_BR.PWM);
			print = 1;
			//printf("%f, %f, %f, %f\n", Motor1_FR.Omega - Motor1_FR.Target_Omega, Motor2_FL.Target_Omega- Motor2_FL.Omega,  Motor3_BL.Omega - Motor3_BL.Target_Omega, Motor4_BR.Target_Omega - Motor4_BR.Omega);
			//printf("%f\n", Angle);
			//printf("%f, %f, %f, ",ax,ay,az);
			//printf("%f, %f\n", acc_test_theta_x, acc_test_theta_y);
//			sum_qwer += atan2(-Trans_X, Trans_Y);
//			qwer++;
//			printf("%f\n", sum_qwer/qwer);
			//printf("%f, %f, %f\n",Trans_X,Trans_Y,Trans_Z);
			//printf("%f, %f\n", sum_qwer/qwer, atan2(-ay, ax*sin(sum_qwer/qwer) + az*cos(sum_qwer/qwer)));
			//printf("%f\n", sqrt(Trans_X*Trans_X + Trans_Y*Trans_Y + Trans_Z*Trans_Z));
			//printf("%f\n", atan2(Trans_Y, Trans_X)*180/acos(-1.0));
			//printf("%f\n", Angle_Acc_Now*180/acos(-1.0));
			//printf("%f, %f\n", Trans_X, Trans_Y);
			//printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor1_FR.Omega, Motor1_FR.Needed_Theta - Motor1_FR.Theta, Motor1_FR.S1, Current_Time, Time_DeAcc, Motor1_FR.Needed_Omega_Acc);
			//printf("%f, %f, %f, %f\n", Motor1_FR.Error_Theta_temp, Motor2_FL.Error_Theta_temp, Motor3_BL.Error_Theta_temp, Motor4_BR.Error_Theta_temp);
			//printf("%f, %f, %f\n", Trans_X, Trans_Y, Angle_Acc_Now*180/acos(-1.0));
			//printf("%f\n", Angle);
			//printf("%f, %f\n", Motor1_FR.E3_Location, Motor2_FL.E3_Location);
			//printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega, Motor1_FR.Omega, Motor2_FL.Omega,Motor3_BL.Omega,Motor4_BR.Omega, Motor4_BR.E2_Speed);
	//printf("%f\n", Motor4_BR.Omega);
			//printf("%f, %f, %f, %f, %f, %f, %f, %f,", Motor1_FR.Target_Omega, Motor2_FL.Target_Omega, Motor3_BL.Target_Omega, Motor4_BR.Target_Omega, Motor1_FR.Omega, Motor2_FL.Omega,Motor3_BL.Omega,Motor4_BR.Omega);
			printf("%f\n", Angle);
		}
		

#endif
	
#if 0
	num_of_time++;
	Get_Encoder();	
	Calculate_Pulse();
	
	
	if (num_of_time >= 10)
	{
		num_of_time = 1;
		
		Calculate_Omega();
		
		Set_Speed_Wheel(&Motor2_FL, 3000);
		Set_Speed_Wheel(&Motor1_FR, 3000);
		
//		PWM_Test();
//		PID_Cascade();
//		PID_Test();
// 		printf("%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d\n", Motor2_FL.Needed_Omega, Motor2_FL.PWM,Motor2_FL.Encoder, Motor2_FL.Omega,Motor2_FL.n, Motor2_FL.Pulse_Count,Motor2_FL.E2_Location,Motor2_FL.Integral_Location,Motor2_FL.Target_Omega,Motor2_FL.E2_Speed,Motor2_FL.Integral_Speed);	
//		printf("%d, %d, %d, %d, %d\n", sum[0], sum[1], sum[2], sum[3], Motor1_FR.PWM);
//		printf("%f, %d, %d\n", Motor2_FL.Omega, Motor2_FL.Encoder, Motor2_FL.Pulse_Count);
//		printf("%f\n", Motor3_BL.Omega);
		printf("%f, %f, %f, %f, %d\n", Motor1_FR.Omega, Motor2_FL.Omega, Motor3_BL.Omega, Motor4_BR.Omega, Motor1_FR.PWM);
//		printf("%d, %d\n", Motor2_FL.Pulse_Count, Motor1_FR.Pulse_Count);
	}
	
	
	if (num_of_time == 10)
	{
		
	}

	
	Get_n();
#endif
	
#if 0
	num_of_time++;
	if (num_of_time == 10)
	{
		num_of_time = 0;
	Set_Speed_Wheel(&Motor2_FL, 3000);
	Set_Speed_Wheel(&Motor1_FR, 3000);
	speed_test();
	}
#endif
}

unsigned short int www = 0;

void Pit_Hnadler_Camera(void)
{
		Get_Image();
		Fushi();
		Gauss_Filter();
		Binaryzation();
		Tangle_Cal();			
}

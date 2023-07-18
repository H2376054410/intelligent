#include "include.h"
#include "gyro.h"

float temp_gyro = 0, temp_angle = 0;

//float Gyro_Record[700] = {0};

float Time_Gyro = 0, B_Hat = 0.000550f, A_Hat = 0.284232f;
float Sigma_XY = 0, Sigma_XX = 0, Sigma_Y = 0, X_Bar = 0, Y_Bar = 0;
unsigned short int n_gyro = 0;
unsigned short int flag_01 = 1, flag_02 = 1;

unsigned short int Flag_Init_Gyro = 1;//初始化去零漂用
unsigned short int Time_Init_Gyro = 0;
float Error_Gyro_X, Error_Gyro_Y, Error_Gyro_Z;
float Error_Gyro_X_2, Error_Gyro_Y_2, Error_Gyro_Z_2;

LOW_Flilter x_flilter,y_flilter,z_flilter;
LOW_Flilter ax_flilter,ay_flilter,az_flilter;
COMPLEMENTARY_FILTER Gx_Filter, Gy_Filter, Gz_Filter;


int timeflag = 0;
float x_degree,y_degree,z_degree,x_degree1,y_degree1,z_degree1;
float recipNorm;
float q0=1;
float q1=0;
float q2=0;
float q3=0;
float Vx,Vy,Vz=0;
float ex,ey,ez,ex1,ey1,ez1,ex2,ey2,ez2=0;
float accex,accey,accez,accex_P,accex_D,accez_P,accez_D,accey_P,accey_D=0;
float ax,ay,az=0;
float gx,gy,gz=0;
float dt=0.005;
float V_roll,V_pitch,V_yaw=0;
float acc_roll,acc_pitch,acc_yaw=0;
float P[2][2]={{1,0},{0,1}};
float Q[2][2]={{0.025,0},{0,0.025}};
float R[2][2]={{0.3,0},{0,0.3}};
float Kk[2][2]={{1,0},{0,1}};
float K_roll,K_pitch,K_yaw=0;
float Angle = 0;

float Trans_X, Trans_Y, Trans_Z, Trans_AX, Trans_AY, Trans_AZ;
float Angle_Acc = 0, Angle_Acc_Now = 0, Angle_Acc_Former = 0;

//float ax_former, ay_former, az_former;

/*编码器低通滤波结果变量*/
float fFL,fBL,fBR,fFR=0;
short int already_meter=0;
short int speed_meter=0;

void Acc_Filter(void)
{
}

void Matrix(void)
{
	Trans_Y = cosz*(ax*cosy - az*siny) + sinz*(ax*sinx*siny + ay*cosx + az*sinx*cosy);
	Trans_X = sinz*(ax*cosy - az*siny) - cosz*(ax*sinx*siny + ay*cosx + az*sinx*cosy);
	Trans_Z = ax*cosx*siny - ay*sinx + az*cosx*cosy;
}

void Gyro_Test(void)
{
//	if (1 == flag_01)
//	{
//		flag_01 = 0;
//		Flag_Init_Gyro = 3;
//		//float *Gyro_Record = (float *)malloc(700 * sizeof(float));
//	}
//	imu660ra_get_gyro();
//	n_gyro++;
//	temp_angle += ((float)imu660ra_gyro_z) / 16.4f * dt;
//	if (n_gyro >= 200 && n_gyro < 600)
//	{
//		//printf("6");
//		Gyro_Record[n_gyro] = temp_angle;
//	}
//	else if (n_gyro >= 600)
//	{
//		if (1 == flag_02)
//		{
//			flag_02 = 0;
//			X_Bar = (200.0f + 600.0f - 1.0f)/2.0f;
//			for (unsigned short int i_temp = 200; i_temp < 600; i_temp++)
//			{
//				//printf("%f\n", Gyro_Record[i_temp]);
//				Sigma_Y += Gyro_Record[i_temp];
//				Sigma_XX += (200 + i_temp) * (200 + i_temp);
//				Sigma_XY += (200 + i_temp) * Gyro_Record[i_temp];
//			}
//			Y_Bar = Sigma_Y / 400.0f;
//			B_Hat = (Sigma_XY - 400.f * X_Bar * Y_Bar) / (Sigma_XX - 400.0f * X_Bar * X_Bar);
//			A_Hat = Y_Bar - B_Hat * X_Bar;
//			//printf("nnn");
//		}
//	}
}

void Gyro_Cal_2(void)//提前测量零漂法，须与Gyro_Test()搭配使用
{
#if 1
	n_gyro++;
	imu660ra_get_gyro();
	temp_angle += ((float)imu660ra_gyro_z) / 16.4f * dt;
	Angle = (B_Hat * n_gyro + A_Hat);
#endif
}


void Gyro_Cal(void)//开局测量消误差法
{
#if 1
	//timeflag++;
		  //imu660ra_get_acc();                                                         // 获取 IMU660RA 的加速度测量数值
      imu660ra_get_gyro();
//				K_roll += (float)imu660ra_gyro_z / 16.4f * dt;
//				printf("%f, %f\n", Angle, K_roll);
//				
			if (1 == Flag_Init_Gyro)
			{
				if (Time_Init_Gyro >= 200)
				{
					Error_Gyro_Z += imu660ra_gyro_z;
				}	
				Time_Init_Gyro++;
				
				if (400 == Time_Init_Gyro)
				{
					Flag_Init_Gyro = 0;
					Error_Gyro_Z /= 200.0f;
				}
			}
			else if (2 == Flag_Init_Gyro)
			{
				Time_Init_Gyro++;
				temp_gyro += ((float)imu660ra_gyro_z - Error_Gyro_Z) * dt;
				if (600 == Time_Init_Gyro)
				{
					Flag_Init_Gyro = 0;
					Error_Gyro_Z_2 = temp_gyro/200.0f;
				}
			}
			else if (0 == Flag_Init_Gyro)
			{
				gz=Complementary_Flilter(((float)imu660ra_gyro_z - Error_Gyro_Z)/16.4f, 0.1, &Gz_Filter);
				temp_angle += (gz * dt);
				Angle = temp_angle + Error_Gyro_Z_2;
				
			}
			
			//printf("%f\n", Angle);
#endif
}

void Gyro_Initialize(void)
{
//		timeflag=0;
//		imu660ra_gyro_x=0;//g是角速度
//		imu660ra_gyro_y=0;
//		imu660ra_gyro_z=0;//横向偏动
//		imu660ra_acc_x=0;//a是加速度
//		imu660ra_acc_y=0;
//		imu660ra_acc_z=0;
//		ax=0;
//		ay=0;
//		az=0;
////	if(timeflag>=1000)
////	{
////		q0+=0.021f;
////		q1-=0.036f;
////		q2-=0.035f;
////		q3-=0.035f;
////	}
//				gx=0;
//				gy=0;
//				gz=0;
//				
//				
////				x_degree+=(gx);
////				y_degree+=(gy);
////				z_degree+=gz*5.5f;

//	//2.将陀螺仪的值转化为弧度
//	
//				imu660ra_gyro_x=0;
//				imu660ra_gyro_y=0;
//				imu660ra_gyro_z=0;
//				
//	//3.对加速度值进行归一化
//				recipNorm=0;
//				ax=0;
//				ay=0;
//				az=0;
//	//			printf("\r\n %f %f %f",ax,ay,az);
//	//4.提取姿态矩阵中的重力分量
//				Vx=0;
//				Vy=0;
//				Vz=0;
//	//5.求姿态误差
//				ex=0;
//				ey=0;
//				ez=0;
//				ex1=0;
//				ey1=0;
//				ez1=0;
//				ex2=0;
//				ey2=0;
//				ez2=0;
//	//6.对误差进行积分
//				accex_P=0;
//				accey_P=0;
//				accez_P=0;
//				accex=0;
//				accey=0;
//				accez=0;
//				accex_D=0;
//				accey_D=0;
//				accez_D=0;
//	//7.互补滤波
//				gx=0;
//				gy=0;
//				gz=0;
//	//8.修正角速度值并解四元数微分方程
//				gx=0;
//				gy=0;
//				gz=0;
//				
//				q0=0;
//				q1=0;
//				q2=0;
//				q3=0;

	//9.四元数归一化
//				recipNorm=0;
				q0=0;
				q1=0;
				q2=0;
				q3=0;
	//10.转化为欧拉角
	      x_degree1=0;
				y_degree1=0;
				z_degree1=0;
				
				
				Angle = 0;

}

void Acc_Cal(void)
{}
	
float invSqrt(float x)
{
	float halfx=0.5f*x;
	float y=x;
	long i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
  return y;
}
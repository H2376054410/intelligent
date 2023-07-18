#include "zf_common_headfile.h"
#include "include.h"

float target_omega = 0;
short int temp_test = 0;
int sum[4] = {0, 0, 0, 0};


void L_Test(void)
{
		position_uart4_flag = 1;
//	if (Flag_If_Received_Yellow_Line == 0)
//	{
//		receive_finished4('L');//这里要有给Art发送消息
//		printf("开始：我已发送L！\n");
//	}
	
	
	while (1)
	{

		//printf("Tell_Art_To_Search_For_Yellow_Line: uart4_receive_finished = %d\n", uart4_receive_finished);
		
		if (Flag_Finish == 1)
			break;
		
		if (Flag_If_Received_Yellow_Line == 0)
		{
//			if (uart4_receive_finished == 1)
//			{
				//system_delay_ms(200);
				receive_finished4('L');//这里要有给Art发送消息
				printf("我已发送L！\n");
//			}
		}
		
		while (!uart4_receive_finished)
		{}
			
		uart4_receive_finished = 0;
		
		if (Flag_If_Received_Yellow_Line == 1)
		{
			//printf("I receiced huangxian_jiaodu = %f\n", huangxian_jiaodu);
			Flag_If_Received_Yellow_Line = 0;
			
//			
//			receive_finished4('L');//这里要有给Art发送消息
//			printf("我已发送L！\n");
//			if (NXPPP.a != 200.0f)
//			{
			temp_angle = huangxian_jiaodu;//就把接收到的角度放在huangxian_jiaodu里吧，另外这个功能需要拉出来单独测试一下
			Times_Of_Yellow_Correction++;
//			}
			huangxian_jiaodu = 0;
//			#if 1
//			Current_X = 35.5f;//如果Art处理的快的话可以考虑顺便把横坐标也矫正了？
//			#endif
			printf("temp_angle = %f\n", temp_angle);
			
			break;
//			if (NXPPP.a != 200.0f)
//				break;
		}

		
		//printf("Motor1_FR.Omega = %f\n", Motor1_FR.Omega);
		if (Current_X < 9 && Current_X > 2)//如果什么都没检测到，也退出
		{
			break;
		}
	}

}



void Location_Correction(void)
{
#if 0
	Flag_Run_State = 1;
	Status = Status_13;
	Flag_Straight_2 = 1;
	Flag_Straight_3 = 0;
	
	while (1)
	{
		printf("%f, %f\n", change_position[0], change_position[1]);
		if (Flag_Finish == 1)
			printf("I am finished!");
	}
#endif 
	while (1)
		if (uart4_receive_finished == 1)//如果接收到了Art发来的图片坐标
		{
			Flag_If_Recieved_Picture = 1;
			uart4_receive_finished = 0;
			Interrupt_To_Pick_Picture();
			Flag_If_Recieved_Picture = 0;
		}
			

	
}
//void Dianqu_Test(void)
//{
//	Flag_Start = 1;
//	while (Current_Time != 2)
//		Set_Speed_Wheel(&Motor1_FR, 2000);
//	while (Current_Time != 4)
//		Set_Speed_Wheel(&Motor1_FR, -2000);
//	Set_Speed_Wheel(&Motor1_FR, 0);
//	
//	while (Current_Time != 6)
//		Set_Speed_Wheel(&Motor2_FL, 2000);
//	while (Current_Time != 8)
//		Set_Speed_Wheel(&Motor2_FL, -2000);
//	Set_Speed_Wheel(&Motor2_FL, 0);
//	
//	while (Current_Time != 10)
//		Set_Speed_Wheel(&Motor3_BL, 2000);
//	while (Current_Time != 12)
//		Set_Speed_Wheel(&Motor3_BL, -2000);
//	Set_Speed_Wheel(&Motor3_BL, 0);
//	
//	while (Current_Time != 14)
//		Set_Speed_Wheel(&Motor4_BR, 2000);
//	while (Current_Time != 16)
//		Set_Speed_Wheel(&Motor4_BR, -2000);
//	Set_Speed_Wheel(&Motor4_BR, 0);
//}

void PWM_Test(void)
{
	Set_Speed_Wheel(&Motor2_FL, Motor2_FL.PWM);
	Set_Speed_Wheel(&Motor1_FR, Motor1_FR.PWM);
	Set_Speed_Wheel(&Motor3_BL, Motor3_BL.PWM);
	Set_Speed_Wheel(&Motor4_BR, Motor4_BR.PWM);
	temp_test++;
	if (temp_test == 500)
	{
		temp_test = 0;
		Motor1_FR.PWM += 500;
		Motor2_FL.PWM += 500;
		Motor3_BL.PWM += 500;
		Motor4_BR.PWM += 500;
		
		sum[0] /= 100;
		sum[1] /= 100;
		sum[2] /= 100;
		sum[3] /= 100;
	}
	else if (400 == temp_test)
	{
		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
		sum[3] = 0;
	}
	else if (temp_test > 400)
	{
		sum[0] += Motor1_FR.Omega;
		sum[1] += Motor2_FL.Omega;
		sum[2] += Motor3_BL.Omega;
		sum[3] += Motor4_BR.Omega;
	}
}

void PID_Test(void)
{
	Pid_Speed_Test(&Motor1_FR);
	Pid_Speed_Test(&Motor2_FL);
	Pid_Speed_Test(&Motor3_BL);
	Pid_Speed_Test(&Motor4_BR);
	
	Set_Speed_Wheel(&Motor2_FL, Motor2_FL.PWM);
	Set_Speed_Wheel(&Motor1_FR, Motor1_FR.PWM);
	Set_Speed_Wheel(&Motor3_BL, Motor3_BL.PWM);
	Set_Speed_Wheel(&Motor4_BR, Motor4_BR.PWM);

	temp_test++;
	if (1000 == temp_test)
	{
		temp_test = 0;
		target_omega += 5;
		sum[0] /= 100;
		sum[1] /= 100;
		sum[2] /= 100;
		sum[3] /= 100;
	}
	else if (900 == temp_test)
	{
		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
		sum[3] = 0;
	}
	else if (temp_test > 900)
	{
		sum[0] += Motor1_FR.PWM;
		sum[1] += Motor2_FL.PWM;
		sum[2] += Motor3_BL.PWM;
		sum[3] += Motor4_BR.PWM;
	}
}	

void PID_Cascade_Test(void)
{
	if (num_of_time % 2)
	{
		Pid_Location(&Motor2_FL);
//		Pid_Location(&Motor1_FR);
//		Pid_Location(&Motor3_BL);
//		Pid_Location(&Motor4_BR);		
	}
		Pid_Speed(&Motor2_FL);
//		Pid_Speed(&Motor1_FR);
//		Pid_Speed(&Motor3_BL);
//		Pid_Speed(&Motor4_BR);
		
		Set_Speed_Wheel(&Motor2_FL, Motor2_FL.PWM);
//		Set_Speed_Wheel(&Motor1_FR, Motor1_FR.PWM);
//		Set_Speed_Wheel(&Motor3_BL, Motor3_BL.PWM);
//		Set_Speed_Wheel(&Motor4_BR, Motor4_BR.PWM);
}

void Pid_Location_Test(Motor_DataTypedef* Motor)
{
	Motor->E2_Location = (Motor->Needed_Pulse_Count - Motor->Pulse_Count)/100.0f;
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
	LimitMax(Motor->Target_Omega, Max_Omega);
	Motor->E1_Location = Motor->E2_Location;
}

void Pid_Speed_Test(Motor_DataTypedef* Motor)
{
	Motor->E2_Speed = target_omega - Motor->Omega;
//	if (Motor->E2_Speed >= -Speed_DeadZone && Motor->E2_Speed <= Speed_DeadZone)//闭环死区
//	{
//		Motor->E1_Speed = 0;
//		Motor->E2_Speed = 0;
//		Motor->Integral_Speed = 0;
//	}
	
//	if(Motor->E2_Speed > -Max_Err_Speed && Motor->E2_Speed < Max_Err_Speed)//积分分离
//  {
		Motor->Integral_Speed += Motor->E2_Speed;  
        /*积分范围限定，防止积分饱和*/
		LimitMax(Motor->Integral_Speed, Max_Integral_Speed);
//  }

	Motor->PWM = (Motor->Kp_Speed * Motor->E2_Speed + Motor->Kd_Speed * (Motor->E2_Speed - Motor->E1_Speed) + Motor->Ki_Speed * Motor->Integral_Speed);
	LimitMax(Motor->PWM, Max_PWM);
	Motor->E1_Speed = Motor->E2_Speed;
}

void speed_test(void)
{


//		Motor1_FR.Omega = Motor1_FR.Pulse_Count / 5;
//		
	
	
	
	Motor2_FL.Encoder = encoder_get_count(ENCODER_FL);                  				// 获取编码器计数
	Motor3_BL.Encoder = encoder_get_count(ENCODER_BL);                          // 获取编码器计数
	Motor4_BR.Encoder = -encoder_get_count(ENCODER_BR);
	Motor1_FR.Encoder = -encoder_get_count(ENCODER_FR);
	
//	Motor2_FL.Pulse_Count += Motor2_FL.Encoder;
//	Motor1_FR.Pulse_Count += Motor1_FR.Encoder;
//	
//	Motor2_FL.Encoder = Alpha * Motor2_FL.Encoder + (1 - Alpha) * Motor2_FL.Pulse_Count_Former;
//	Motor2_FL.Pulse_Count_Former = Motor2_FL.Encoder;
	Motor2_FL.Omega = 300 * 3.1416f * Motor2_FL.Encoder / (7.0f * 512 * Interrupt_Ms);
	Motor1_FR.Omega = 300 * 3.1416f * Motor1_FR.Encoder / (7.0f * 512 * Interrupt_Ms);
	
//	Motor2_FL.Omega = 3000 * 3.1416f * Motor2_FL.Encoder / (7.0f * 512 * Interrupt_Ms);
//	Motor1_FR.Omega = 3000 * 3.1416f * Motor1_FR.Encoder / (7.0f * 512 * Interrupt_Ms);
	
	printf("%f\n", Motor2_FL.Omega);

	
	encoder_clear_count(ENCODER_FL);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_FR);                                       // 清空编码器计数
	encoder_clear_count(ENCODER_BL);
	encoder_clear_count(ENCODER_BR);


}
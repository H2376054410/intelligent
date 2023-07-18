//motor1对应4，motor2对应1.motor3对应2，motor4对应3
#include "zf_common_headfile.h"
#include "include.h"


int main(void)
{

	//float *coordernation=(float*)malloc(50*sizeof(float));//单个坐标点
/************************************************************************************************************************************************************************************************/
	clock_init(SYSTEM_CLOCK_600M);  // 不可删除
	motor_init();//初始化电机
	debug_init();                   // 调试端口初始化
	wireless_uart_init();
	imu660ra_init();
	encoder_init(5);//意思是1毫秒进一次中断
	//Camera_Init(20);//意思是20毫秒进一次中断
/************************************************************************************************************************************************************************************************/
	//printf("无线串口好用");
	while (1)
	{
		if (Flag_Init_Gyro == 0)
			break;
	}//等待陀螺仪矫正完成
/************************************************************************************************************************************************************************************************/
	//receive_finished4('L');//串口4是黄线
		//receive_finished('M');
	
//	while (1)
//	{
//		printf("%f\n", debug_uart_data);
//	}
	//Servo_Run();
	//putttt();
	//Run_Dot();
//	Go_Home();
//		
//	Open();
//	Flag_Run_State = 0;//置0的话没有位姿矫正
//	Spin(-180);
//	Wait();
//		
//		
//	Open();
//	Flag_Run_State = 0;//置0的话没有位姿矫正
//	Set_Location_XOY(-2.0f, 0, 0.5);
//	Wait();


//	Tell_Art_To_Search_For_Yellow_Line();
//	receive_finished4('L');
//	receive_finished('L');
//	printf("我已发送L\n");
//	
	
	Recognize_All_Dot();
	Lets_Start();
	Place_Picture();
	Go_Back();

	while (1)
	{
		Set_Speed_Wheel(&Motor2_FL, 9000);
	}


	
	while(1)
	{
		
	}
}




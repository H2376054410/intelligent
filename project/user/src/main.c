//motor1��Ӧ4��motor2��Ӧ1.motor3��Ӧ2��motor4��Ӧ3
#include "zf_common_headfile.h"
#include "include.h"


int main(void)
{

	//float *coordernation=(float*)malloc(50*sizeof(float));//���������
/************************************************************************************************************************************************************************************************/
	clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
	motor_init();//��ʼ�����
	debug_init();                   // ���Զ˿ڳ�ʼ��
	wireless_uart_init();
	imu660ra_init();
	encoder_init(5);//��˼��1�����һ���ж�
	//Camera_Init(20);//��˼��20�����һ���ж�
/************************************************************************************************************************************************************************************************/
	//printf("���ߴ��ں���");
	while (1)
	{
		if (Flag_Init_Gyro == 0)
			break;
	}//�ȴ������ǽ������
/************************************************************************************************************************************************************************************************/
	//receive_finished4('L');//����4�ǻ���
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
//	Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
//	Spin(-180);
//	Wait();
//		
//		
//	Open();
//	Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
//	Set_Location_XOY(-2.0f, 0, 0.5);
//	Wait();


//	Tell_Art_To_Search_For_Yellow_Line();
//	receive_finished4('L');
//	receive_finished('L');
//	printf("���ѷ���L\n");
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




#include "include.h"

unsigned short int Flag_Left_Or_Right = 1;
unsigned short int Num_Of_Times_Sum = 0, Num_Of_Times = 0;
unsigned short int Flag_If_Recieved_Art_Yellow_Line = 0, Flag_If_Sent_Message_To_Art_Yellow_Line = 0;
unsigned short int Flag_If_Recieved_Picture = 0;
short int Times_Of_Yellow_Correction = 0;

float Distance_Y = 0;
float Angle_Art = 0;
float X_Art = 0, Y_Art = 0;

short int Sign;


void Recognize_All_Dot(void)
{
	receive_finished('M');
	printf("���ѷ���M\n");
	while (!second_flag)
	{}
}


void Lets_Start(void)
{
	
	Num_Of_Times_Sum = (int)(4.8f / Width_Of_View) + ((4.8f / Width_Of_View - (int)(4.8f / Width_Of_View) > 0) ? 1 : 0);//����ȡ��
	Distance_Y = 4.8f / Num_Of_Times_Sum;//S��·��Y�᷽����Ҫ�ľ��룬0.2���ܵõ�
	
	Open();
	Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
	Set_Location_XOY(0, 0.2, 0.7);//�߳����ⴥ����ʱ
	Wait();
	

	while (1)
	{
		
		Sign = ((Flag_Left_Or_Right == 0) ? (1) : (-1));//���÷��ţ�����Ϊ����
		
		
		Open();
		Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
		Set_Location_XOY(-Sign * 7.0f, 0, 0.3);//����/����
		//Wait();		
		while (1)
		{
			
			
			//printf("Flag_Finish = %d\n", Flag_Finish);
			if (Flag_Finish == 1)//������ȼ�����������ˣ���break
			{
				Flag_If_Sent_Message_To_Art_Yellow_Line = 0;
				Flag_If_Recieved_Art_Yellow_Line = 0;//��ʼ����־λ
				break;
			}
			
			
			
		
			
			
//			if ((Current_X > 33 || Current_X < 2) && Num_Of_Times != 0)//�������30�񣬼�6�ף���һ����6�ף���������������͸�Art����Ϣ���������߽��߲��������ͽǶ���Ϣ�Խ��������Լ�������
//			{
//				//position_uart4_flag = 1;
//				//printf("Current_X = %f\n", Current_X);
//				if (Times_Of_Yellow_Correction - 1 != Num_Of_Times)
//				{
//					Tell_Art_To_Search_For_Yellow_Line();
//				}
//				//printf("Yellowline??\n");
//			}		

			
			
			
			
			
			
			position_uart4_flag = 0;
			//printf("Flag_If_Received_Picture_Dot=%d, Current_X=%f\n", Flag_If_Received_Picture_Dot, Current_X);
			if (Flag_If_Received_Picture_Dot == 1 && Current_X > 2 && Current_X < 33)//������յ���Art������ͼƬ����
			{
				//Flag_If_Received_Picture_Dot = 0;
				printf("098");
				//Flag_If_Recieved_Picture = 1;
				uart4_receive_finished = 0;
				Interrupt_To_Pick_Picture();
				//Flag_If_Recieved_Picture = 0;
			}
				
			
		
		}
		Wait();
		
		
		
		Num_Of_Times++;
		if (Num_Of_Times == Num_Of_Times_Sum)//������е��ж�������
		{
			//printf("Num_Of_Times_Sum = %d, Num_Of_Times = %d\n", Num_Of_Times_Sum, Num_Of_Times);
			break;
		}
			
		
		//printf("???");
		Open();
		Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
		Set_Location_XOY(0, Distance_Y, 0.5);//��ǰ��
		Wait();
		
		Flag_Left_Or_Right = Flag_Left_Or_Right ^ 0x1;//��־λȡ��
	}
}


void Tell_Art_To_Search_For_Yellow_Line(void)
{
	
	
	position_uart4_flag = 1;
//	if (Flag_If_Received_Yellow_Line == 0)
//	{
//		receive_finished4('L');//����Ҫ�и�Art������Ϣ
//		printf("��ʼ�����ѷ���L��\n");
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
				receive_finished4('L');//����Ҫ�и�Art������Ϣ
				printf("���ѷ���L��\n");
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
//			receive_finished4('L');//����Ҫ�и�Art������Ϣ
//			printf("���ѷ���L��\n");
//			if (NXPPP.a != 200.0f)
//			{
			temp_angle = huangxian_jiaodu;//�Ͱѽ��յ��ĽǶȷ���huangxian_jiaodu��ɣ��������������Ҫ��������������һ��
			Times_Of_Yellow_Correction++;
//			}
			huangxian_jiaodu = 0;
//			#if 1
//			Current_X = 35.5f;//���Art����Ŀ�Ļ����Կ���˳��Ѻ�����Ҳ�����ˣ�
//			#endif
			printf("temp_angle = %f\n", temp_angle);
			
			break;
//			if (NXPPP.a != 200.0f)
//				break;
		}

		
		//printf("Motor1_FR.Omega = %f\n", Motor1_FR.Omega);
		if (Current_X < 33 && Current_X > 2)//���ʲô��û��⵽��Ҳ�˳�
		{
			break;
		}
	}
}





void Interrupt_To_Pick_Picture(void)
{
	//Flag_Run_State = 0;//��0�Ļ�û��λ�˽����������䵽ʱ���ע�͵�
	Flag_000 = 0;
	Decelerate();
	Flag_000 = 1;
	Wait();//���� + λ�˽���
	
	
	printf("Success!\n");
	
//	while (1)
//	{
//		
//	}
//	
//	Dot_Correction();//����������������ֵ���õ�����û��
//	
//	Servo_Run();
//	putttt();//��ͼƬ�����ﻹ��Ҫ��
	
	
	Open();
	Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
	Set_Location_XOY(0, (Y_Temp - Current_Y) * 0.2f, 0.4f);
	Wait();//��ȥ����ֱ�ػ�ȥ��
	
	
	
	Open();
	Flag_Run_State = 0;//��0�Ļ�û��λ�˽���
	if (Flag_Left_Or_Right == 0)
	{
		Set_Location_XOY((0 - Current_X) * 0.2f, 0, 0.3);
	}
	else if (Flag_Left_Or_Right == 1)
	{
		Set_Location_XOY((36.0f - Current_X) * 0.2f, 0, 0.3);
	}//��������Ŀ��λ�ã������ر�Ե��
	printf("�����ߣ�");
	//Wait();
}





void Dot_Correction(void)
{
	unsigned short int i = 0;
	while (1)
	{
		if (fabs(Current_X - zuobiaodui[i][0]) < 3 && fabs(Current_Y - zuobiaodui[i][1]) < 3)//������������֮��
		{
			Current_X = zuobiaodui[i][0];
			Current_Y = zuobiaodui[i][1];
			break;
		}
		else
		{
			i++;
		}
		
		if (0)//���ﻹӦ��������������˾�break
			break;
	}
}

void Decelerate(void)//�˹�����Ҫ���ԣ�
{
	
	//Flag_Straight_1 = 1;
	
	
	Omega_X_Max = Motor1_FR.Omega;//sin��cos
	
	Motor1_FR.Needed_Omega_Max = Sign * Motor1_FR.Omega;
	Motor2_FL.Needed_Omega_Max = -Sign * Motor1_FR.Omega;
	Motor3_BL.Needed_Omega_Max = Sign * Motor1_FR.Omega;
	Motor4_BR.Needed_Omega_Max = -Sign * Motor1_FR.Omega;

	Motor2_FL.Pulse_Count = 0;
	Motor1_FR.Pulse_Count = 0;
	Motor4_BR.Pulse_Count = 0;
	Motor3_BL.Pulse_Count = 0;
	
	Motor2_FL.Pulse_Count_Former = 0;
	Motor1_FR.Pulse_Count_Former = 0;
	Motor4_BR.Pulse_Count_Former = 0;
	Motor3_BL.Pulse_Count_Former = 0;//�Ȱ�Theta����
	
	Motor2_FL.Theta = 0;
	Motor1_FR.Theta = 0;
	Motor4_BR.Theta = 0;
	Motor3_BL.Theta = 0;//�Ȱ�Theta����

	

	Motor1_FR.Needed_Theta = Sign * 9.0f;//����9���¶�Ӧ10���֣����ٶȼ���0��Ҫ��10cm
	Motor2_FL.Needed_Theta = -Sign * 9.0f;
	Motor3_BL.Needed_Theta = Sign * 9.0f;
	Motor4_BR.Needed_Theta = -Sign * 9.0f;

	
	Motor1_FR.Needed_Omega_Acc = Motor1_FR.Needed_Omega_Max * Motor1_FR.Needed_Omega_Max / (2.0f * Motor1_FR.Needed_Theta);
	Motor2_FL.Needed_Omega_Acc = Motor2_FL.Needed_Omega_Max * Motor2_FL.Needed_Omega_Max / (2.0f * Motor2_FL.Needed_Theta);
	Motor3_BL.Needed_Omega_Acc = Motor3_BL.Needed_Omega_Max * Motor3_BL.Needed_Omega_Max / (2.0f * Motor3_BL.Needed_Theta);
	Motor4_BR.Needed_Omega_Acc = Motor4_BR.Needed_Omega_Max * Motor4_BR.Needed_Omega_Max / (2.0f * Motor4_BR.Needed_Theta);
	
	
	Motor1_FR.S1 = Motor1_FR.Needed_Theta;
	Motor2_FL.S1 = Motor2_FL.Needed_Theta;
	Motor3_BL.S1 = Motor3_BL.Needed_Theta;
	Motor4_BR.S1 = Motor4_BR.Needed_Theta;

	Status = Status_13;
	Flag_Run_Normal = 1;
	Flag_Run_State = 1;
	Flag_Acc = 0;
	Flag_DeAcc = 1;
	
}

void Place_Picture(void)
{
	Go_To(18.0f, 27.0f);
	//������Ҫд��ͼƬ���µ�������
	Go_To(37.0f, 13.0f);
	//������Ҫд��ͼƬ���µ�������
	Go_To(18.0f, -1.0f);
	//������Ҫд��ͼƬ���µ�������
	Go_To(-1.0f, 13.0f);
	//������Ҫд��ͼƬ���µ�������
}

void Go_Back(void)
{
	Go_To(1.0f, 1.0f);
}
#include "include.h"
int camera_situation=0;//0代表此处是全黑的
int i=0;
int j=0;
float p0,p1,m1,m2=0;
int sum_cal,mid1=0;
int num1,num2=0;
float nnum1 = 60,nnum2 = 94;

uint8 after_image[120][188]={0};
float gaosi[3][3]={{0.0751,0.1238,0.0751},{0.1238,0.2042,0.1238},{0.0751,0.1238,0.0751}};
float tidu[120][188],tidux[120][188],tiduy[120][188]={0};
float angle=0;
float gaosi_image[120][188]={0};


void Camera_Init(unsigned short int Time)
{
	   while(1)
    {
        if(mt9v03x_init())
				{
           


				printf("Zong Zuan Feng init error!\n");                                           // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
				while(1)
					{}
				}

				else
				{
						printf("Zong Zuan Feng init successful!\n");

						break;
				}
    }
		pit_ms_init(PIT_CH1, Time);                                                   // 初始化 PIT1 为周期中断 time 周期

		interrupt_set_priority(PIT_IRQn, 1);                                    // 设置 PIT1 对周期中断的中断优先级为 0
    interrupt_global_enable(0);
}

void Get_Image(void)
{
//	while (1)
//	{
//		if(mt9v03x_finish_flag)
//		{

			for(i=0;i<120;i++)
			{
				for(j=0;j<188;j++)
				{
						after_image[i][j]=mt9v03x_image[i][j];//存储接收到的图片
				}
			}
			mt9v03x_finish_flag = 0;
//			break;
//		}
//	}
}

void Gauss_Filter(void)
{
	for(i=0;i<118;i++)
	{
		for(j=0;j<186;j++)
		{
//							 gaosi_image[i][j]+=after_image[i][j]*gaosi[0][0];
//							gaosi_image[i][j]+=after_image[i][j]*gaosi[0][0];
//							gaosi_image[i][j+1]+=after_image[i][j+1]*gaosi[0][1];
//							gaosi_image[i][j+2]+=after_image[i][j+2]*gaosi[0][2];
//							gaosi_image[i+1][j]+=after_image[i+1][j]*gaosi[1][0];
//							gaosi_image[i+1][j+1]+=after_image[i+1][j+1]*gaosi[1][1];
//							gaosi_image[i+1][j+2]+=after_image[i+1][j+2]*gaosi[1][2];
//							gaosi_image[i+2][j]+=after_image[i+2][j]*gaosi[2][0];
//							gaosi_image[i+2][j+1]+=after_image[i+2][j+1]*gaosi[2][1];
//							gaosi_image[i+2][j+2]+=after_image[i+2][j+2]*gaosi[2][2];
						gaosi_image[i+1][j+1]=after_image[i][j]*gaosi[0][0]+after_image[i][j+1]*gaosi[0][1]+after_image[i][j+2]*gaosi[0][2]
																 +after_image[i+1][j]*gaosi[1][0]+after_image[i+1][j+1]*gaosi[1][1]+after_image[i+1][j+2]*gaosi[1][2]
																 +after_image[i+2][j]*gaosi[2][0]+after_image[i+2][j+1]*gaosi[2][1]+after_image[i+2][j+2]*gaosi[2][2];
		}
	}

					for(i=0;i<118;i++)
					{
						for(j=0;j<186;j++)
						{
							tiduy[i+1][j+1]=gaosi_image[i+2][j]+2*gaosi_image[i+2][j+1]+gaosi_image[i+2][j+2]-gaosi_image[i][j]-2*gaosi_image[i][j+1]-gaosi_image[i][j+2];
							tidux[i+1][j+1]=gaosi_image[i][j+2]+2*gaosi_image[i+1][j+2]+gaosi_image[i+2][j+2]-gaosi_image[i][j]-2*gaosi_image[i+1][j]-2*gaosi_image[i+2][j];
							tidu[i+1][j+1]=sqrt(tiduy[i+1][j+1]*tiduy[i+1][j+1]+tidux[i+1][j+1]*tidux[i+1][j+1]);

						}
					}
}


void Fushi(void)
{
							for(i=0;i<118;i+=3)
							{
								for(j=0;j<186;j+=3)
								{
									if(tidu[i][j+2]>=150||tidu[i+1][j+2]>=150||tidu[i+1][j+1]>=150)

									{
										tidu[i][j+2]=255;
										tidu[i+1][j+2]=255;
										tidu[i+1][j+1]=255;
									}
								}
							}
							
							for(i=0;i<118;i+=3)
							{
								for(j=0;j<186;j+=3)
								{
									if(tidu[i][j+2]<=50||tidu[i+1][j+2]<=50||tidu[i+1][j+1]<=50)

									{
										tidu[i][j+2]=0;
										tidu[i+1][j+2]=0;
										tidu[i+1][j+1]=0;
									}
								}
							}
}

void Binaryzation(void)
{
							for(i=0;i<120;i++)
							{
								for(j=0;j<188;j++)
								{
									after_image[i][j]=(uint8)tidu[i][j];
									gaosi_image[i][j]=0;
									if(after_image[i][j]>=180)
									{
									after_image[i][j]=255;
									}else
									{
										after_image[i][j]=0;
									}
								}
							
							}
}

void Send_To_PC(void)
{
	camera_send_image(WIRELESS_UART_INDEX, (const uint8 *)after_image, MT9V03X_IMAGE_SIZE);//向电脑发送图像，赛场上不需要
}

void Tangle_Cal(void)
{
							for(i=5;i<115;i++)
							{
								for(j=8;j<180;j++)
								{
									if(after_image[i][j]>=254)
									{
											num1+=i;
											num2+=j;
											sum_cal++;
										
									}
								}
							}
							nnum1=(float)num1;
							nnum2=(float)num2;
					
							nnum1/=(float)sum_cal;
							nnum2/=(float)sum_cal;
							if(sum_cal>=750)
							{
							  camera_situation=1;
							}
							//printf("%f, %f, %d\n",nnum1,nnum2, camera_situation);
//							angle=atanf((nnum2-94)/(120-nnum1));
							num1=0;
							num2=0;
							sum_cal=0;
//	printf("\r\n %f \r\n",angle*57.0f);
//		camera_send_image(WIRELESS_UART_INDEX, (const uint8 *)after_image, MT9V03X_IMAGE_SIZE);//向电脑发送图像，赛场上不需要

		

}
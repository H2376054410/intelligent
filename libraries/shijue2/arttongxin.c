#include "arttongxin.h"
#include "stdlib.h"
#include "math.h"
#include "zf_common_headfile.h"
#include "include.h"

union formal NXPP;
uint8 *union_p=&NXPP.biaozhi[0];
uint8 *union_look=&NXPP.biaozhi[0];
uint8 union_buffer[4]={0};
int union_flag=255;
int situation=1;
int second_flag=0;
int flag=0;
int flag1=0;
int location_num=0;
int send_time=0;//0代表没发送过，1代表已经发送过一次了
int motion_situation=1;//0代表静止，1代表运动
float image_kind=0;
union Test tep;
int flag_recognize_finished = 0;




//t54,c43,s53，M识别A4，T发送坐标点，C图像分类,S发送图像分类结果
//void Recognize_Dot(float *coordernation)
//{
//		system_delay_ms(500);
//	  receive_finished(0x4D);
//	//printf("asd");
//	receive(&(tep.biaozhi));
//	//printf("qwe");
//	coordernation[coordernation_num]=tep.a;
//if(coordernation[coordernation_num]==100.0f)	
//{
//	receive_finished(0x54);
//}
////printf("zxc");
//	while(1){
//		receive(&(tep.biaozhi));
//		coordernation[coordernation_num]=tep.a;
//		//printf("vbn");
//		if(coordernation[coordernation_num]==100.0f){
//		//printf("uio");
//		break;
//		}
//		zuobiaodui[flag][flag1]=coordernation[coordernation_num];
//		if(flag1==0){
//		flag1=1;
//		flag=flag;}
//		else if(flag1==1)
//		{
//			flag++;
//			flag1=0;		
//		}
//		printf("Dots \n",coordernation[coordernation_num]);
//		
//			coordernation_num++;
//		receive_finished(0x54);//向art发送

//	}
//}


//void Recognize_Image(float *coordernation)
//{
//		while(1)
//	{
//		
//		receive_finished(0x43);
//		//printf("\r\n c \r\n");
//		receive(&(tep.biaozhi));
//		coordernation[coordernation_num]=tep.a;
//		
//		
//		if(coordernation[coordernation_num]==100.0f)
//		{
//		receive_finished(0x53);
//			printf("\r\n s \r\n");
//		receive(&(tep.biaozhi));
//		coordernation[coordernation_num]=tep.a;
//		printf("Result of Recocnition is %f\n",coordernation[coordernation_num]);
//		coordernation_num++;
//		}
//		else 
//			printf("%f\n", coordernation[coordernation_num]);

//	}
//	printf("\r\n Second Step Finished \r\n");

//}



int check_if_exists(int num, int arr[], int arr_size) {
    for (int i = 0; i < arr_size; i++) {
        if (arr[i] == num) {
            return 1;   // 如果找到了相等的数，返回1
        }
    }
    return 0;           // 如果没有找到相等的数，返回0
}





//void receive(uint8 *p)
//{
//	int i=0;
//	for(i=0;i<=3;i++)
//{
//	*p++=uart_read_byte(UART_1);
//}
//}

//void receive_finished(uint8 p)
//{
////			uart_write_byte(UART_1, 0x0D);
////			uart_write_byte(UART_1, 0x0A);
//			uart_write_byte(UART_1, p);
////			uart_write_byte(UART_1, 0x0D);
////			uart_write_byte(UART_1, 0x0A);

//}


int min_idd(float *distance,int all)
{
	int k=1;
	int i=0;
	int j=0;
	for(i=0;i<all;i++)
	{
		k=1;
		for(j=0;j<all;j++){
		if(distance[i]>distance[j])
		{
			k=0;
			break;
		}
		}
		if(k==1)
		{
			return i;
		}
	}
	return 0;
}
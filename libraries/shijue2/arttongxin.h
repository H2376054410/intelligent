#ifndef __ARTTONGXIN_H
#define __ARTTONGXIN_H
#include "zf_common_headfile.h"

extern union formal NXPP;
extern uint8 *union_p;
extern uint8 *union_look;
extern uint8 union_buffer[4];
extern int union_flag;
extern int situation;
extern int second_flag;
extern int flag;
extern int flag1;
extern int location_num;
extern int send_time;//0代表没发送过，1代表已经发送过一次了
extern int motion_situation;//0代表静止，1代表运动
extern float image_kind;
extern int flag_recognize_finished;


extern void Recognize_Dot(float *coordernation);
extern void Recognize_Image(float *coordernation);
extern void receive_finished(uint8 p);
extern void receive(uint8 *p);
extern int min_idd(float *distance,int all);
extern void distance_count(float *x1,float *y1,float *x2,float *y2,float *distance);
extern int check_if_exists(int num, int arr[], int arr_size);
extern int findMinExcept(float arr[], int n, int except[], int m);
union Test
{
	float a;
	int b;
	uint8 biaozhi;
};
union formal
{
	float a;
	int b;
	uint8 biaozhi[4];
};

#endif
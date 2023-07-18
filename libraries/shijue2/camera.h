#ifndef __CAMERA_H
#define __CAMERA_H

extern int camera_situation;
extern int i;
extern int j;
extern float p0,p1,m1,m2;
extern int sum_cal,mid1;
extern int num1,num2;
extern float nnum1,nnum2;
extern unsigned char after_image[120][188];
extern float gaosi[3][3];
extern float tidu[120][188],tidux[120][188],tiduy[120][188];
extern float angle;
extern float gaosi_image[120][188];



extern void Camera_Init(unsigned short int Time);
extern void Get_Image(void);
extern void Gauss_Filter(void);
extern void Fushi(void);
extern void Binaryzation(void);
extern void Tangle_Cal(void);
extern void Send_To_PC(void);
#endif
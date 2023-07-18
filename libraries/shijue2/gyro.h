#ifndef __GYRO_H
#define __GYRO_H

#define theta_x 0.015968
#define theta_y 0.153241
#define theta_z 2.34675

#define sinx 0.015967
#define cosx 0.999873
#define siny 0.152642
#define cosy 0.988282
#define sinz 0.713753
#define cosz -0.700397

#define X1 -0.709120
#define Y1 -0.697281
#define Z1 0.0872

extern float Gyro_Record[700];
extern float K_roll,K_pitch,K_yaw,x_degree,y_degree,z_degree,x_degree1,y_degree1,z_degree1, Angle;
extern float ax, ay, az;
extern float Trans_X, Trans_Y, Trans_Z, Angle_Acc, Angle_Acc_Former, Angle_Acc_Now, Trans_AX, Trans_AY, Trans_AZ;
extern unsigned short int Flag_Init_Gyro;
extern float B_Hat, A_Hat;

extern float Time_Gyro, temp_angle;
extern float Sigma_XY, Sigma_XX, Sigma_Y, X_Bar, Y_Bar;
extern unsigned short int n_gyro;
extern unsigned short int flag_01, flag_02;
//extern float Gyro_Record[700];




extern void Gyro_Initialize();
extern void Matrix(void);
extern void Acc_Cal(void);
extern float invSqrt(float x);
extern void Gyro_Cal(void);
extern void Gyro_Cal_2(void);
extern void Gyro_Test(void);

#endif
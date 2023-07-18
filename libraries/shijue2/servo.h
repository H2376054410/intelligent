#define dianci_gpio                 (PWM1_MODULE3_CHA_B10)
#define SERVO_MOTOR_PWM             (PWM4_MODULE2_CHA_C30)                          // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ            (50 )                                           // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300

#define SERVO_MOTOR_L_MAX_UNDER           (90 )                                           // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX_UNDER           (193)                                           // ���������϶�����Χ �Ƕ�


#define SERVO_MOTOR_L_MAX_ABOVE           (110)                                           // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX_ABOVE           (188)                                           // ���������϶�����Χ �Ƕ�

	


#define duoji_under (PWM4_MODULE3_CHA_C31) 
#define duoji_mid   (PWM4_MODULE2_CHA_C30)
#define duoji_above (PWM1_MODULE3_CHA_B10)


#define duoji_under_min 0
#define duoji_under_max 180
#define duoji_mid_min 0
#define duoji_mid_max 180
#define duoji_above_min 0
#define duoji_under_max 180


#define duoji_under_start 0
#define duoji_mid_start 90
#define duoji_above_start 90

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)50)*(0.5+(float)(x)/90.0))
	







extern float servo_motor_duty_duty;
extern float servo_motor_dir; // �������״̬
extern float servo_motor_dir1;
extern int count_servo,count_under;

extern void all_pwm_init(void);
extern void Servo_Run(void);
extern void putttt(void);
extern void put_under(void);
extern void put_up(void);
extern void duandian(void);
extern void fangche(void);
extern void duoji_down(void);
extern void duoji_up(void);
extern void dct_open(void);
extern void dct_close(void);










#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif


extern float servo_motor_dut;                                                  // ��������Ƕ�

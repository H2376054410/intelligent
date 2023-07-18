#include "include.h"

float servo_motor_duty= 90;                                                  // 舵机动作角度
float servo_motor_duty_duty=0;
float servo_motor_dir = 1; // 舵机动作状态
float servo_motor_dir1 = 1;
int count_servo,count_under=0;
//31是下面的，最小是90，       30是上面的，最小是70
unsigned short int Flag_Servo_Finish = 0;



//舵机引脚确定，C30,C31，B10，舵机顺序为中间，下面，上面
int duoji_under_nowdata=duoji_under_start;
int duoji_mid_nowdata=duoji_mid_start;
int duoji_above_nowdata=duoji_above_start;



void all_pwm_init(void)
{
		pwm_init(PWM4_MODULE2_CHA_C30, 50, 0);
	  pwm_init(PWM4_MODULE3_CHA_C31, 50, 0);
	  pwm_init(PWM2_MODULE0_CHA_C6, 50, 0);
	  pwm_init(PWM2_MODULE0_CHB_C7, 50, 0);
	  pwm_init(PWM2_MODULE1_CHA_C8, 50, 0);
	  pwm_init(PWM2_MODULE1_CHB_C9, 50, 0);
		pwm_init(PWM4_MODULE0_CHA_B24, 300, 0);
		pwm_init(PWM4_MODULE1_CHA_B25, 50, 0);
	  pwm_init(PWM1_MODULE3_CHA_B10, 50, 0);
		pwm_init(PWM2_MODULE2_CHA_C10, 50, 0);
	  pwm_init(PWM2_MODULE2_CHB_C11, 50, 0);
	  pwm_init(PWM2_MODULE3_CHB_C19, 50, 0);

}
void duoji_down(void)
{
		pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(115));
	  pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(190));
	  pwm_set_duty(PWM1_MODULE3_CHA_B10, (uint32)SERVO_MOTOR_DUTY(90));
	  system_delay_ms(1500);
}
void duoji_up(void)
{
		pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(0));
	  system_delay_ms(500);
	  pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(60));
	  pwm_set_duty(PWM1_MODULE3_CHA_B10, (uint32)SERVO_MOTOR_DUTY(150));
    system_delay_ms(1000);
}	
void dct_open(void)
{
	pwm_set_duty(PWM4_MODULE0_CHA_B24,10000);
}
void dct_close(void)
{
  pwm_set_duty(PWM4_MODULE0_CHA_B24,0);
}



void Servo_Run(void)
{
//printf("开始吸");
	pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(servo_motor_duty));
	pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(servo_motor_duty_duty));
    while(1)
    {
				if (Flag_Servo_Finish)
				{
					Flag_Servo_Finish = 0;
					break;
				}

        pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(servo_motor_duty));

        if(servo_motor_dir)
        {
            servo_motor_duty ++;
            if(servo_motor_duty >= SERVO_MOTOR_R_MAX_UNDER)
            {
                servo_motor_dir = 0x00;
												while(1)
												{
														pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(servo_motor_duty_duty));
														if(servo_motor_dir1)
														{
															//printf("开始吸111");
																servo_motor_duty_duty ++;
																if(servo_motor_duty_duty >= SERVO_MOTOR_R_MAX_ABOVE)
																{
																servo_motor_dir1 = 0x00;
																pwm_set_duty(dianci_gpio, 10000);
																printf(" 此处应该吸起 ");
																system_delay_ms(3000); 
																count_servo++;
																break;
																}
														}
														system_delay_ms(2);
												}
            }
        }
        else
        {
            servo_motor_duty --;
            if(servo_motor_duty <= SERVO_MOTOR_L_MAX_UNDER)
            {
												while(1)
												{
														pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(servo_motor_duty_duty));
														if(servo_motor_dir1!=0x01)
														{
																servo_motor_duty_duty --;
																if(servo_motor_duty_duty <= SERVO_MOTOR_L_MAX_ABOVE)
																{
																	servo_motor_dir1 = 0x01;
																	count_servo++;
																	Flag_Servo_Finish = 1;
																	break;
																}
														}
														system_delay_ms(2);
												}
			//				pwm_set_duty(dianci_gpio, 0);
              servo_motor_dir = 0x01;
            }
        }
        system_delay_ms(2);    				
    }
}
void put_up(void)
{
		pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_L_MAX_UNDER));
	  pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_L_MAX_ABOVE));
		system_delay_ms(1000);

}
void put_under(void)
{
		pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_R_MAX_UNDER));
	  pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_R_MAX_ABOVE));
		system_delay_ms(1000);
}

void duandian(void)
{
	system_delay_ms(1000);
	pwm_set_duty(dianci_gpio, 0);
	printf("此处断电了");
	system_delay_ms(1000);
	pwm_set_duty(dianci_gpio, 10000);
 printf("此处重新上电");

}

void putttt(void)
{
	int above=70;
	int under=90;
	while(1)
	{
		under++;
		pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(under));
		if(under>=150)
		{
				while(1)
				{
					above++;
					pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(above));
					if(above>=150)
						break;
				}
				if(under>=150&&above>=150)
				{
					break;
				}
				

		}
	
	}
//	pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(150));
//	pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(180));
	system_delay_ms(1000);
	duandian();
	
	pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(90));
	pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(70));
	pwm_set_duty(dianci_gpio, 0);

}
void fangche(void)
{
	pwm_set_duty(PWM4_MODULE3_CHA_C31, (uint32)SERVO_MOTOR_DUTY(90));
	pwm_set_duty(PWM4_MODULE2_CHA_C30, (uint32)SERVO_MOTOR_DUTY(80));
  system_delay_ms(3000);
	pwm_set_duty(dianci_gpio, 0);
}
	


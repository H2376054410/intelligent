#include "include.h"

///**
//  * @brief  λ��PID�㷨ʵ��
//  * @param  actual_val:ʵ��ֵ
//  * @note   ��
//  * @retval ͨ��PID���������
//  */
//#define LOC_DEAD_ZONE 60 /*λ�û�����*/
//#define LOC_INTEGRAL_START_ERR 200 /*���ַ���ʱ��Ӧ����Χ*/
//#define LOC_INTEGRAL_MAX_VAL 800   /*���ַ�Χ�޶�����ֹ���ֱ���*/
//float location_pid_realize(PID *pid, float actual_val)
//{
//    /*����Ŀ��ֵ��ʵ��ֵ�����*/
//    pid->err = pid->target_val - actual_val;

//    /* �趨�ջ����� */
//    if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }

//    /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
//    if(pid->err > -LOC_INTEGRAL_START_ERR && pid->err < LOC_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;  
//        /*���ַ�Χ�޶�����ֹ���ֱ���*/
//        if(pid->integral > LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = LOC_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -LOC_INTEGRAL_MAX_VAL;
//        }
//    }   

//    /*PID�㷨ʵ��*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd * (pid->err - pid->err_last);

//    /*����*/
//    pid->err_last = pid->err;

//    /*���ص�ǰʵ��ֵ*/
//    return pid->output_val;
//}

///**
//  * @brief  �ٶ�PID�㷨ʵ��
//  * @param  actual_val:ʵ��ֵ
//  * @note   ��
//  * @retval ͨ��PID���������
//  */
//#define SPE_DEAD_ZONE 5.0f /*�ٶȻ�����*/
//#define SPE_INTEGRAL_START_ERR 100 /*���ַ���ʱ��Ӧ����Χ*/
//#define SPE_INTEGRAL_MAX_VAL 260   /*���ַ�Χ�޶�����ֹ���ֱ���*/
//float speed_pid_realize(PID *pid, float actual_val)
//{
//    /*����Ŀ��ֵ��ʵ��ֵ�����*/
//    pid->err = pid->target_val - actual_val;

//    /* �趨�ջ����� */
//    if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }

//    /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
//    if(pid->err > -SPE_INTEGRAL_START_ERR && pid->err < SPE_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;  
//        /*���ַ�Χ�޶�����ֹ���ֱ���*/
//        if(pid->integral > SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = SPE_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -SPE_INTEGRAL_MAX_VAL;
//        }
//    }   

//    /*PID�㷨ʵ��*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd *(pid->err - pid->err_last);

//    /*����*/
//    pid->err_last = pid->err;

//    /*���ص�ǰʵ��ֵ*/
//    return pid->output_val;
//}

////���ڶ�ʱ���Ļص�����
//void AutoReloadCallback()
//{
//    static unsigned int location_timer = 0;    // λ�û�����

//    static int encoderNow = 0;    /*��ǰʱ���ܼ���ֵ*/
//    static int encoderLast = 0;   /*��һʱ���ܼ���ֵ*/
//    int encoderDelta = 0; /*��ǰʱ������һʱ�̱������ı仯��*/
//    float actual_speed = 0;  /*ʵ�ʲ���ٶ�*/
//    int actual_speed_int = 0;

//    int res_pwm = 0;/*PID����õ���PWMֵ*/
//    static int i=0;

//    /*��1����ȡ��������ֵ*/
//    encoderNow = read_encoder() + EncoderOverflowCnt*ENCODER_TIM_PERIOD;/*��ȡ��ǰ���ۼ�ֵ*/
//    encoderDelta = encoderNow - encoderLast; /*�õ��仯ֵ*/
//    encoderLast = encoderNow;/*�����ϴε��ۼ�ֵ*/

//    /*��2��λ��PID���㣬�õ�PWM����ֵ*/
//    if ((location_timer++ % 2) == 0)
//    {
//        float control_val = 0;   /*��ǰ����ֵ*/

//        /*λ��PID����*/
//        control_val = location_pid_realize(&pid_location, encoderNow);  

//        /*Ŀ���ٶ�ֵ����*/
//        speed_val_protect(&control_val);

//        /*�趨�ٶ�PID��Ŀ��ֵ*/
//        set_pid_target(&pid_speed, control_val);    
//    }

//    /* ת��(1����ת����Ȧ)=��λʱ���ڵļ���ֵ/�ֱܷ���*ʱ��ϵ��, �ٳ�60��Ϊ1����ת����Ȧ */
//    actual_speed = (float)encoderDelta / TOTAL_RESOLUTION * 10 * 60;

//    /*��3���ٶ�PID���㣬�õ�PWM����ֵ*/
//    actual_speed_int = actual_speed;
//    res_pwm = pwm_val_protect((int)speed_pid_realize(&pid_speed, actual_speed));

//    /*��4��PWM���Ƶ��*/
//    set_motor_rotate(res_pwm);

//    /*��5�������ϴ�����λ����ʾ*/
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &encoderNow, 1);   /*��ͨ��1����ʵ�ʵĵ����λ�á�ֵ*/
//}
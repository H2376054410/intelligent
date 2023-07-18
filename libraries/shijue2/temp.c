#include "include.h"

///**
//  * @brief  位置PID算法实现
//  * @param  actual_val:实际值
//  * @note   无
//  * @retval 通过PID计算后的输出
//  */
//#define LOC_DEAD_ZONE 60 /*位置环死区*/
//#define LOC_INTEGRAL_START_ERR 200 /*积分分离时对应的误差范围*/
//#define LOC_INTEGRAL_MAX_VAL 800   /*积分范围限定，防止积分饱和*/
//float location_pid_realize(PID *pid, float actual_val)
//{
//    /*计算目标值与实际值的误差*/
//    pid->err = pid->target_val - actual_val;

//    /* 设定闭环死区 */
//    if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }

//    /*积分项，积分分离，偏差较大时去掉积分作用*/
//    if(pid->err > -LOC_INTEGRAL_START_ERR && pid->err < LOC_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;  
//        /*积分范围限定，防止积分饱和*/
//        if(pid->integral > LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = LOC_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -LOC_INTEGRAL_MAX_VAL;
//        }
//    }   

//    /*PID算法实现*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd * (pid->err - pid->err_last);

//    /*误差传递*/
//    pid->err_last = pid->err;

//    /*返回当前实际值*/
//    return pid->output_val;
//}

///**
//  * @brief  速度PID算法实现
//  * @param  actual_val:实际值
//  * @note   无
//  * @retval 通过PID计算后的输出
//  */
//#define SPE_DEAD_ZONE 5.0f /*速度环死区*/
//#define SPE_INTEGRAL_START_ERR 100 /*积分分离时对应的误差范围*/
//#define SPE_INTEGRAL_MAX_VAL 260   /*积分范围限定，防止积分饱和*/
//float speed_pid_realize(PID *pid, float actual_val)
//{
//    /*计算目标值与实际值的误差*/
//    pid->err = pid->target_val - actual_val;

//    /* 设定闭环死区 */
//    if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }

//    /*积分项，积分分离，偏差较大时去掉积分作用*/
//    if(pid->err > -SPE_INTEGRAL_START_ERR && pid->err < SPE_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;  
//        /*积分范围限定，防止积分饱和*/
//        if(pid->integral > SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = SPE_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -SPE_INTEGRAL_MAX_VAL;
//        }
//    }   

//    /*PID算法实现*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd *(pid->err - pid->err_last);

//    /*误差传递*/
//    pid->err_last = pid->err;

//    /*返回当前实际值*/
//    return pid->output_val;
//}

////周期定时器的回调函数
//void AutoReloadCallback()
//{
//    static unsigned int location_timer = 0;    // 位置环周期

//    static int encoderNow = 0;    /*当前时刻总计数值*/
//    static int encoderLast = 0;   /*上一时刻总计数值*/
//    int encoderDelta = 0; /*当前时刻与上一时刻编码器的变化量*/
//    float actual_speed = 0;  /*实际测得速度*/
//    int actual_speed_int = 0;

//    int res_pwm = 0;/*PID计算得到的PWM值*/
//    static int i=0;

//    /*【1】读取编码器的值*/
//    encoderNow = read_encoder() + EncoderOverflowCnt*ENCODER_TIM_PERIOD;/*获取当前的累计值*/
//    encoderDelta = encoderNow - encoderLast; /*得到变化值*/
//    encoderLast = encoderNow;/*更新上次的累计值*/

//    /*【2】位置PID运算，得到PWM控制值*/
//    if ((location_timer++ % 2) == 0)
//    {
//        float control_val = 0;   /*当前控制值*/

//        /*位置PID计算*/
//        control_val = location_pid_realize(&pid_location, encoderNow);  

//        /*目标速度值限制*/
//        speed_val_protect(&control_val);

//        /*设定速度PID的目标值*/
//        set_pid_target(&pid_speed, control_val);    
//    }

//    /* 转速(1秒钟转多少圈)=单位时间内的计数值/总分辨率*时间系数, 再乘60变为1分钟转多少圈 */
//    actual_speed = (float)encoderDelta / TOTAL_RESOLUTION * 10 * 60;

//    /*【3】速度PID运算，得到PWM控制值*/
//    actual_speed_int = actual_speed;
//    res_pwm = pwm_val_protect((int)speed_pid_realize(&pid_speed, actual_speed));

//    /*【4】PWM控制电机*/
//    set_motor_rotate(res_pwm);

//    /*【5】数据上传到上位机显示*/
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &encoderNow, 1);   /*给通道1发送实际的电机【位置】值*/
//}
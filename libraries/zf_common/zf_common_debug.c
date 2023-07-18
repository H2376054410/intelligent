/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_common_debug
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "fsl_pwm.h"
#include "zf_common_fifo.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_uart.h"
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "communicate.h"
#include "include.h"
#include "stdlib.h"
#if DEBUG_UART_USE_INTERRUPT                                                    // ������� debug uart �����ж�
uint8                       debug_uart_buffer[DEBUG_RING_BUFFER_LEN];           // ���ݴ������
uint8                       debug_uart_data;
fifo_struct                 debug_uart_fifo;
#endif
#if DEBUG_UART_USE_INTERRUPT                                                    // ������� debug uart �����ж�
uint8                       debug_uart_output_buffer[DEBUG_RING_BUFFER_LEN];           // ���ݴ������
uint8                       debug_uart4_data;
fifo_struct                 debug_uart4_fifo;
#endif
uint32 what=4;
uint32 *positionn=&what;


//union formal NXPP;
//uint8 *union_p=&NXPP.biaozhi[0];
//uint8 *union_look=&NXPP.biaozhi[0];
//uint8 union_buffer[4]={0};
//int union_flag=255;
int zhentou_flag=0;
//int situation=1;
//int second_flag=0;

//int flag=0;
//int flag1=0;
//int location_num=0;
//int send_time=0;//0����û���͹���1�����Ѿ����͹�һ����
//int motion_situation=1;//0����ֹ��1�����˶�
//float image_kind=0;
static debug_output_struct  debug_output_info;
static volatile uint8       zf_debug_init_flag = 0;
static volatile uint8       zf_debug_assert_enable = 1;

union formal NXPPP;
uint8 *union_pp=&NXPP.biaozhi[0];
uint8 *union_lookk=&NXPP.biaozhi[0];
uint8 union_bufferr[4]={0};
int union_flagg=0;
unsigned short int Flag_Uart_Temp = 0;

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ����ʱ���� �� 120MHz ����һ����ʱ�� ����Ƭ����Ҫ���ݸ���ʱ������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_delay();
// ��ע��Ϣ     ���������ļ��ڲ����� �û����ù�ע Ҳ�����޸�
//-------------------------------------------------------------------------------------------------------------------
static void debug_delay (void)
{
    vuint32 loop_1 = 0, loop_2 = 0;
    for(loop_1 = 0; loop_1 <= 0xFF; loop_1 ++)
    {
        for(loop_2 = 0; loop_2 <= 0xFFFF; loop_2 ++)
        {
            __NOP();
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug �������� ��Ҫ�Ƿ�ֹ���Ժ�����ź�ά�ֶ�����Ӳ��ʧ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_protective_handler();
// ��ע��Ϣ     ���������ļ��ڲ����� �û����ù�ע Ҳ�����޸�
//-------------------------------------------------------------------------------------------------------------------
static void debug_protective_handler (void)
{
    pwm_set_duty(PWM1_MODULE0_CHB_D13, 0);
    pwm_set_duty(PWM1_MODULE0_CHA_D12, 0);
    pwm_set_duty(PWM1_MODULE1_CHB_D15, 0);
    pwm_set_duty(PWM1_MODULE1_CHA_D14, 0);
    pwm_set_duty(PWM1_MODULE2_CHB_D17, 0);
    pwm_set_duty(PWM1_MODULE2_CHA_D16, 0);
    pwm_set_duty(PWM1_MODULE3_CHB_B11, 0);
    pwm_set_duty(PWM1_MODULE3_CHA_B10, 0);
    
    
    pwm_set_duty(PWM2_MODULE0_CHB_C7, 0);
    pwm_set_duty(PWM2_MODULE0_CHA_C6, 0);
    pwm_set_duty(PWM2_MODULE1_CHB_C9, 0);
    pwm_set_duty(PWM2_MODULE1_CHA_C8, 0);
    pwm_set_duty(PWM2_MODULE2_CHB_C11, 0);
    pwm_set_duty(PWM2_MODULE2_CHA_C10, 0);
    pwm_set_duty(PWM2_MODULE3_CHB_B1, 0);
    pwm_set_duty(PWM2_MODULE3_CHA_B0, 0);
    
    pwm_set_duty(PWM4_MODULE0_CHA_B24, 0);
    pwm_set_duty(PWM4_MODULE1_CHA_B25, 0);
    pwm_set_duty(PWM4_MODULE2_CHA_C30, 0);
    pwm_set_duty(PWM4_MODULE3_CHA_C31, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ��������ӿ�
// ����˵��     *str        ��Ҫ������ַ���
// ���ز���     void
// ʹ��ʾ��     debug_uart_str_output("Log message");
// ��ע��Ϣ     ���������ļ��ڲ����� �û����ù�ע Ҳ�����޸�
//-------------------------------------------------------------------------------------------------------------------
static void debug_uart_str_output (const char *str)
{
    uart_write_string(DEBUG_UART_INDEX, str);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ����ӿ�
// ����˵��     *type       log ����
// ����˵��     *file       �ļ���
// ����˵��     line        Ŀ������
// ����˵��     *str        ��Ϣ
// ���ز���     void
// ʹ��ʾ��     debug_output("Log message", file, line, str);
// ��ע��Ϣ     ���������ļ��ڲ����� �û����ù�ע Ҳ�����޸�
//-------------------------------------------------------------------------------------------------------------------
static void debug_output (char *type, char *file, int line, char *str)
{
    char *file_str;
    vuint16 i = 0, j = 0;
    vint16 len_origin = 0;
    vint16 show_len = 0;
    vint16 show_line_index = 0;
    len_origin = strlen(file);

    char output_buffer[256];
    char file_path_buffer[64];

    if(debug_output_info.type_index)
    {
        debug_output_info.output_screen_clear();
    }

    if(zf_debug_init_flag)
    {
        if(debug_output_info.type_index)
        {
            // ��Ҫ���н��ļ���·�����������
            // <���������·�� ֻ���һ��Ŀ¼ ���� src/main.c>
            // ��� line : xxxx
            debug_output_info.output_screen(0, show_line_index ++, type);

            file_str = file;
            len_origin = strlen(file);
            show_len = (debug_output_info.display_x_max / debug_output_info.font_x_size);

            while(*file_str++ != '\0');

            // ֻȡһ��Ŀ¼ ����ļ������̷���Ŀ¼ ���� MDK �Ĺ��̸�Ŀ¼ �ͻ�ֱ�������ǰĿ¼
            for(j = 0; (j < 2) && (len_origin >= 0); len_origin --)             // �������� '/'
            {
                file_str --;
                if((*file_str == '/') || (*file_str == 0x5C))
                {
                    j ++;
                }
            }

            // �ļ�·�����浽������
            if(len_origin >= 0)
            {
                file_str ++;
                sprintf(output_buffer, "file: %s", file_str);
            }
            else
            {
                if(0 == j)
                {
                    sprintf(output_buffer, "file: mdk/%s", file_str);
                }
                else
                {
                    sprintf(output_buffer, "file: %s", file_str);
                }
            }

            // ��Ļ��ʾ·��
            for(i = 0; i < ((strlen(output_buffer) / show_len) + 1); i ++)
            {
                for(j = 0; j < show_len; j ++)
                {
                    if(strlen(output_buffer) < (j + i * show_len))
                    {
                        break;
                    }
                    file_path_buffer[j] = output_buffer[j + i * show_len];
                }
                
                file_path_buffer[j] = '\0';                                     // ĩβ���\0
                
                debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, file_path_buffer);
            }

            // ��Ļ��ʾ�к�
            sprintf(output_buffer, "line: %d", line);
            debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, output_buffer);

            // ��Ļ��ʾ Log ����еĻ�
            if(NULL != str)
            {
                for(i = 0; i < ((strlen(str) / show_len) + 1); i ++)
                {
                    for(j = 0; j < show_len; j ++)
                    {
                        if(strlen(str) < (j + i * show_len))
                        {
                            break;
                        }
                        file_path_buffer[j] = str[j + i * show_len];
                    }
                    
                    file_path_buffer[j] = '\0';                                 // ĩβ���\0
                    
                    debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, file_path_buffer);
                }
            }
        }
        else
        {
            char output_buffer[256];
            memset(output_buffer, 0, 256);
            debug_output_info.output_uart(type);
            if(NULL != str)
            {
                sprintf(output_buffer, "\r\nfile %s line %d: %s.\r\n", file, line, str);
            }
            else
            {
                sprintf(output_buffer, "\r\nfile %s line %d.\r\n", file, line);
            }
            debug_output_info.output_uart(output_buffer);
        }
    }
}

#if DEBUG_UART_USE_INTERRUPT                                                    // �������� ֻ�������ô����жϲű���

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ debug ���λ���������
// ����˵��     *data       �������ݴ�ŵ�����ָ��
// ���ز���     uint32      �������ݵ�ʵ�ʳ���
// ʹ��ʾ��     uint8 data[64]; uint32 len = debug_read_ring_buffer(data);
// ��ע��Ϣ     ��������Ҫ���� DEBUG_UART_USE_INTERRUPT �궨��ſ�ʹ��
//-------------------------------------------------------------------------------------------------------------------
uint32 debug_read_ring_buffer (uint8 *data)
{
    uint32 data_len = sizeof(data);
    fifo_read_buffer(&debug_uart_fifo, data, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug �����жϴ����� isr.c �ж�Ӧ�����жϷ���������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_interrupr_handler();
// ��ע��Ϣ     ��������Ҫ���� DEBUG_UART_USE_INTERRUPT �궨��ſ�ʹ��
//              ���ұ�����Ĭ�Ϸ����� UART1 �Ĵ��ڽ����жϴ���
//-------------------------------------------------------------------------------------------------------------------
float position[25][2]={0};
int order_flag=0;
void debug_interrupr_handler (void)
{
//	if (Flag_Uart_Temp == 0)
//	{
//		Flag_Uart_Temp = 1;

//	int union_flag1=0;
//	int union_flag2=0;
//	printf("tge first");
    if(zf_debug_init_flag)
    {
				uart_query_byte(DEBUG_UART_INDEX, &debug_uart_data);                    // ��ȡ��������
				
				//printf("%x\n",debug_uart_data);
			
			
			if(situation==1)
			{
				if(zhentou_flag==1)
				{
					switch(union_flag)
					{
						case 0:union_buffer[0]=debug_uart_data;
									NXPP.biaozhi[0]=union_buffer[0];
									//Flag_Uart_Temp = 0;
									//printf("\r\n 0==%x \r\n",NXPP.biaozhi[0]);
									union_flag++;break;
						case 1:union_buffer[1]=debug_uart_data;
									NXPP.biaozhi[1]=union_buffer[1];
									//Flag_Uart_Temp = 0;
									//printf("\r\n 1==%x \r\n",NXPP.biaozhi[1]);
									union_flag++;break;
						case 2:union_buffer[2]=debug_uart_data;
									NXPP.biaozhi[2]=union_buffer[2];
									//Flag_Uart_Temp = 0;
									//printf("\r\n 2==%x \r\n",NXPP.biaozhi[2]);
									union_flag++;break;
						case 3:union_buffer[3]=debug_uart_data;
									NXPP.biaozhi[3]=union_buffer[3];
									//Flag_Uart_Temp = 0;
									//printf("\r\n 3==%x \r\n",NXPP.biaozhi[3]);
									printf("\r\n%f �������\r\n",NXPP.a);
									union_flag=0;
									zuobiaodui[flag][flag1]=NXPP.a;
									if(zuobiaodui[flag][flag1]==100.0f)
									{
											second_flag=1;
											situation=2;
							//				printf("\r\n ��%d���˳���һ�׶� \r\n",flag);
											break;
									}
										if(flag1==0)
										{
												flag1=1;
										}else
										{
												
												flag++;
												flag1=0;
										}
									
									break;
						
						default:printf("\r\n error \r\n");break;
					}
			  }
				if(debug_uart_data==0xff&&zhentou_flag==0)
				{
					printf("\r\n ����ռ���������Ϊ255 \r\n");
					union_flag=0;
					//Flag_Uart_Temp = 0;
					zhentou_flag=1;
				}
			}
			else
			{
				switch(union_flag)
				{
					case 0:union_buffer[0]=debug_uart_data;
								NXPP.biaozhi[0]=union_buffer[0];
								union_flag++;break;
					case 1:union_buffer[1]=debug_uart_data;
					      NXPP.biaozhi[1]=union_buffer[1];					
								union_flag++;break;
					case 2:union_buffer[2]=debug_uart_data;
					      NXPP.biaozhi[2]=union_buffer[2];			
								union_flag++;break;
					case 3:union_buffer[3]=debug_uart_data;
								NXPP.biaozhi[3]=union_buffer[3];
								printf("\r\n %x \r\n",NXPP.biaozhi[3]);
								printf("\r\n %f \r\n",NXPP.a);
					      union_flag=0;
								image_kind=NXPP.a;
								printf("\r\n ʶ�𵽵�ͼƬ����Ϊ��%f \r\n",image_kind);
								motion_situation=1;
								send_time=0;				
					      break;
				 default:printf("\r\n error \r\n");break;
				}
			}
        fifo_write_buffer(&debug_uart_fifo, &debug_uart_data, 1);               // ���� FIFO
//			printf("\r\n ��ø�ͷ \r\n");
//		  	zf_log(fifo_read_buffer(&debug_uart_fifo, &debug_uart_output_data, positionn, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS, "fifo_read_buffer error");
//			printf("\r\n %x \r\n",debug_uart_output_data);
//			printf("\r\n ��� \r\n");
    }
//	}
}




float change_position[2]={0};
int change_positionxxx=1;
int position_flag=255;
int position_num=0;
int position_uart4_flag=0;
float huangxian_jiaodu=0;
int huangxian_flag=255;
int huangxian_flag1=255;
int uart4_receive_finished=0;


unsigned short int Flag_If_Received_Picture_Dot = 0;
unsigned short int Flag_If_Received_Yellow_Line = 0;



void debug_UART4_interrupr_handler (void)//λ�˽���
{
	if(zf_debug_init_flag)
  {
		if(position_uart4_flag==0)		//�ж���ͼƬ��⻹�ǻ��߼�⣬������ͼƬ���
		{
			uart_query_byte(UART_4, &debug_uart4_data);                    // ��ȡ��������'
			
			
			if(position_flag==1)
			{
				if(change_positionxxx)
				{
					switch(union_flagg)
					{
					 case 0:union_bufferr[0]=debug_uart4_data;
									NXPPP.biaozhi[0]=union_bufferr[0];
									union_flagg++;break;
					 case 1:union_bufferr[1]=debug_uart4_data;
									NXPPP.biaozhi[1]=union_bufferr[1];
									union_flagg++;break;
					 case 2:union_bufferr[2]=debug_uart4_data;
									NXPPP.biaozhi[2]=union_bufferr[2];
									union_flagg++;break;
					 case 3:union_bufferr[3]=debug_uart4_data;
									NXPPP.biaozhi[3]=union_bufferr[3];
									change_position[0]=NXPPP.a / 100.0f;
								 //printf("change_position[0] = %f,  ",NXPPP.a);
									change_positionxxx=0;
									
									union_flagg=0;break;
					 default:printf("\r\n position correctionx error! \r\n");break;
					}
				}
				else
				{
					switch(union_flagg)
					{
					 case 0:union_bufferr[0]=debug_uart4_data;
									NXPPP.biaozhi[0]=union_bufferr[0];
									union_flagg++;break;
					 case 1:union_bufferr[1]=debug_uart4_data;
									NXPPP.biaozhi[1]=union_bufferr[1];
									union_flagg++;break;
					 case 2:union_bufferr[2]=debug_uart4_data;
									NXPPP.biaozhi[2]=union_bufferr[2];
									union_flagg++;break;
					 case 3:union_bufferr[3]=debug_uart4_data;
									NXPPP.biaozhi[3]=union_bufferr[3];
									change_position[1]=-NXPPP.a / 100.0f;
								//printf("change_position[1] = %f\n",NXPPP.a);
									Flag_If_Received_Picture_Dot = 1;
						
						
									//printf("\r\n %f \r\n",NXPPP.a);
									//printf("\r\n ������� \r\n");
									change_positionxxx=1;
									union_flagg=0;
									
									//printf("change_position[1]: uart4_receive_finished = %d\n", uart4_receive_finished);
									uart4_receive_finished=1;
									position_flag=255;
									break;
					 default:printf("\r\n position correctiony error! \r\n");break;
					}
				}
			}
			
			
			if(debug_uart4_data==0xff)
			{
				position_flag=1;
			}
			
		}
		else if(position_uart4_flag==1)
		{
			uart_query_byte(UART_4, &debug_uart4_data);                    // ��ȡ��������'
			
			
			if(debug_uart4_data==0xff)
			{
				huangxian_flag=1;
			}
			
			
			if(huangxian_flag1==1)
			{
				switch(union_flagg)
				{
				 case 0:union_bufferr[0]=debug_uart4_data;
								NXPPP.biaozhi[0]=union_bufferr[0];
								union_flagg++;break;
				 case 1:union_bufferr[1]=debug_uart4_data;
								NXPPP.biaozhi[1]=union_bufferr[1];
								union_flagg++;break;
				 case 2:union_bufferr[2]=debug_uart4_data;
								NXPPP.biaozhi[2]=union_bufferr[2];
								union_flagg++;break;
				 case 3:union_bufferr[3]=debug_uart4_data;
								NXPPP.biaozhi[3]=union_bufferr[3];
					
								if (NXPPP.a == 255.0f)
									printf("NXPPP.a == 255!\n");
					
								if (NXPPP.a != 200.0f && NXPPP.a != 255.0f)
								{
									huangxian_jiaodu=NXPPP.a;
									position_uart4_flag=0;
									Flag_If_Received_Yellow_Line = 1;
								}
								else
									;
								
						
					
								printf("huangxian_jiaodu = %f\n",huangxian_jiaodu);
								
								
								huangxian_flag=255;
								huangxian_flag1=255;
								//printf("uart4_receive_finished = %d\n", uart4_receive_finished);
								uart4_receive_finished=1;
								union_flagg=0;break;
				 default:printf("\r\n position correctionx error! \r\n");break;
				}		
			}
			
			
			if(huangxian_flag==1)
			{
				huangxian_flag1=1;
			}
		 
		 
		}
			
	}
}

#endif

//-------------------------------------------------------------------------     // printf �ض��� �˲��ֲ������û�����
#if defined(__ICCARM__)
#define PUTCHAR_PROTOTYPE int32_t fputc (int32_t ch, FILE *f)
#define GETCHAR_PROTOTYPE int32_t fgetc (FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int32_t __io_putchar (int32_t ch)
#define GETCHAR_PROTOTYPE int32_t __io_getchar ()
#endif

#if defined(__ICCARM__)
PUTCHAR_PROTOTYPE
{
    uart_write_byte(DEBUG_UART_INDEX, (ch & 0xFF));
    return ch;
}

GETCHAR_PROTOTYPE
{
    return uart_read_byte(DEBUG_UART_INDEX);
}
#else
//int32_t fputc (int32_t ch, FILE* f)
//{
//    uart_write_byte(DEBUG_UART_INDEX, (ch & 0xFF));
//    return ch;
//}

int fgetc(FILE *f)
{
    return uart_read_byte(DEBUG_UART_INDEX);
}
int32_t fputc (int32_t ch, FILE* f)
{
    uart_write_byte(WIRELESS_UART_INDEX, (ch & 0xFF));
    return ch;
}

//int fgetc(FILE *f)
//{
//    return uart_read_byte(UART_4);
//}
#endif
//-------------------------------------------------------------------------     // printf �ض��� �˲��ֲ������û�����

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ö���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_assert_enable();
// ��ע��Ϣ     ����Ĭ�Ͽ��� ���鿪������
//-------------------------------------------------------------------------------------------------------------------
void debug_assert_enable (void)
{
    zf_debug_assert_enable = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ö���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_assert_disable();
// ��ע��Ϣ     ����Ĭ�Ͽ��� ��������ö���
//-------------------------------------------------------------------------------------------------------------------
void debug_assert_disable (void)
{
    zf_debug_assert_enable = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ���Դ�����
// ����˵��     pass        �ж��Ƿ񴥷�����
// ����˵��     *file       �ļ���
// ����˵��     line        Ŀ������
// ���ز���     void
// ʹ��ʾ��     zf_assert(0);
// ��ע��Ϣ     �����������ֱ�ӵ��õ� �˲��ֲ������û�����
//              ʹ�� zf_commmon_debug.h �е� zf_assert(x) �ӿ�
//-------------------------------------------------------------------------------------------------------------------
void debug_assert_handler (uint8 pass, char *file, int line)
{
    do
    {
        if(pass || !zf_debug_assert_enable)
        {
            break;
        }

        static uint8 assert_nest_index = 0;

        if(0 != assert_nest_index)
        {
            while(1);
        }
        assert_nest_index ++;

        interrupt_global_disable();
        debug_protective_handler();

        while(1)
        {
            // ���������ת������ͣס��
            // һ����ĺ����������ݳ�����
            // �������Լ����õ� zf_assert(x) �ӿڴ�������

            // ��������� debug_init ��ʼ���� log ���
            // ���ڶ�Ӧ�������ȥ�鿴���ĸ��ļ�����һ�б���

            // ���û�г�ʼ�� debug
            // �ǾͿ������ file ���ַ���ֵ�� line ������
            // �Ǵ�������ļ�·�����ƺͶ�Ӧ��������

            // ��ȥ���Կ�����Ϊʲô��������

            debug_output("Assert error", file, line, NULL);
            debug_delay();
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ������Ϣ������
// ����˵��     pass        �ж��Ƿ񴥷�����
// ����˵��     *str        �������Ϣ
// ����˵��     *file       �ļ���
// ����˵��     line        Ŀ������
// ���ز���     void
// ʹ��ʾ��     zf_log(0, "Log Message");
// ��ע��Ϣ     �����������ֱ�ӵ��õ� �˲��ֲ������û�����
//              ʹ�� zf_commmon_debug.h �е� zf_log(x, str) �ӿ�
//-------------------------------------------------------------------------------------------------------------------
void debug_log_handler (uint8 pass, char *str, char *file, int line)
{
    do
    {
        if(pass)
        {
            break;
        }
        if(zf_debug_init_flag)
        {
            debug_output("Log message", file, line, str);
            //printf("Log message from %s line %d :\"%s\".\r\n", file, line, str);
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug �������Ϣ��ʼ��
// ����˵��     *info       debug �������Ϣ�ṹ��
// ���ز���     void
// ʹ��ʾ��     debug_output_struct_init(info);
// ��ע��Ϣ     �������һ�㲻���û�����
//-------------------------------------------------------------------------------------------------------------------
void debug_output_struct_init (debug_output_struct *info)
{
    info->type_index            = 0;

    info->display_x_max         = 0xFFFF;
    info->display_y_max         = 0xFFFF;

    info->font_x_size           = 0xFF;
    info->font_y_size           = 0xFF;

    info->output_uart           = NULL;
    info->output_screen         = NULL;
    info->output_screen_clear   = NULL;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ����󶨳�ʼ��
// ����˵��     *info       debug �������Ϣ�ṹ��
// ���ز���     void
// ʹ��ʾ��     debug_output_init(info);
// ��ע��Ϣ     �������һ�㲻���û�����
//-------------------------------------------------------------------------------------------------------------------
void debug_output_init (debug_output_struct *info)
{
    debug_output_info.type_index            = info->type_index;

    debug_output_info.display_x_max         = info->display_x_max;
    debug_output_info.display_y_max         = info->display_y_max;

    debug_output_info.font_x_size           = info->font_x_size;
    debug_output_info.font_y_size           = info->font_y_size;
    
    debug_output_info.output_uart           = info->output_uart;
    debug_output_info.output_screen         = info->output_screen;
    debug_output_info.output_screen_clear   = info->output_screen_clear;

    zf_debug_init_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     debug ���ڳ�ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     debug_init();
// ��ע��Ϣ     ��Դ��ʾ��Ĭ�ϵ��� ��Ĭ�Ͻ����жϽ���
//-------------------------------------------------------------------------------------------------------------------
void debug_init (void)
{
    debug_output_struct info;
    debug_output_struct_init(&info);
    info.output_uart = debug_uart_str_output;
    debug_output_init(&info);

    uart_init(
        DEBUG_UART_INDEX,                                                       // �� zf_common_debug.h �в鿴��Ӧֵ
        DEBUG_UART_BAUDRATE,                                                    // �� zf_common_debug.h �в鿴��Ӧֵ
        DEBUG_UART_TX_PIN,                                                      // �� zf_common_debug.h �в鿴��Ӧֵ
        DEBUG_UART_RX_PIN);                                                     // �� zf_common_debug.h �в鿴��Ӧֵ
	    uart_init(
        UART_4,                                                       // �� zf_common_debug.h �в鿴��Ӧֵ
        DEBUG_UART_BAUDRATE,                                                    // �� zf_common_debug.h �в鿴��Ӧֵ
        UART4_TX_C16,                                                      // �� zf_common_debug.h �в鿴��Ӧֵ
        UART4_RX_C17);                                                     // �� zf_common_debug.h �в鿴��Ӧֵ

#if DEBUG_UART_USE_INTERRUPT                                                    // �������� ֻ�������ô����жϲű���
    fifo_init(&debug_uart_fifo, FIFO_DATA_8BIT, debug_uart_buffer, DEBUG_RING_BUFFER_LEN);
    uart_rx_interrupt(DEBUG_UART_INDEX, 1);                                     // ʹ�ܶ�Ӧ���ڽ����ж�
	  uart_rx_interrupt(UART_4, 1);  
#endif
}


/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
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
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_driver_delay
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

#ifndef _zf_driver_delay_h_
#define _zf_driver_delay_h_

#include "zf_common_typedef.h"
#include "zf_common_clock.h"

void        system_delay            (uint32 time, uint32 num);

//====================================================�궨�庯����====================================================
//-------------------------------------------------------------------------------------------------------------------
// �������     system ��ʱ���� ms ����
// ����˵��     time			��Ҫ��ʱ��ʱ�� ms ����
// ���ز���     void
// ʹ��ʾ��     system_delay_ms(100);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
#define     system_delay_ms(time)   system_delay(BOARD_XTAL_FREQ/1000, time)            // ������ʱʱ�� ��λ ms

//-------------------------------------------------------------------------------------------------------------------
// �������     system ��ʱ���� us ����
// ����˵��     time			��Ҫ��ʱ��ʱ�� us ����
// ���ز���     void
// ʹ��ʾ��     system_delay_us(100);
// ��ע��Ϣ     �����ڳ���������ת ����ʱ�������ֵ�߳�һЩ
//-------------------------------------------------------------------------------------------------------------------
#define     system_delay_us(time)   system_delay(BOARD_XTAL_FREQ/1000/1000*time, 1)     // ������ʱʱ�� ��λ us

//====================================================�궨�庯����====================================================

#endif
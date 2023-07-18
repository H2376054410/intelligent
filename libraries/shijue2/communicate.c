#include "include.h"

void receive(unsigned char *p)
{
	int i=0;
	for(i=0;i<=3;i++)
	{
		*p++=uart_read_byte(UART_1);
	}
}

void receive4(uint8 *p)
{
	int i=0;
	for(i=0;i<=3;i++)
	{
		*p++=uart_read_byte(UART_4);
	}
}


void receive_finished(uint8 p)
{
			uart_write_byte(UART_1, p);
}
void receive_finished4(uint8 p)
{
			uart_write_byte(UART_4, p);
}
void receive_finished4l(void)
{
			uart_write_byte(UART_4, 'L');
}
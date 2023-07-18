#include "include.h"

	
	
float zuobiaodui[25][2]={0};
float now_zuobiaodui[2] = {0};
int coordernation_num=0;
int count = 0;
float zoux = 0;
float zouy = 0;


int min_id;
float min_distance = 10000.0f;
int ordered_path[30] = {-1};
float ordered_zuobiaodui[30][2] = {0};





//	float* distance = (float* )malloc(sizeof(float) * (flag + 1));
int already_have[100] = {-1};
float distance[50] = {0};


void Greedy_Algorithm(short int flag)//45行还有些问题
{
	int i = 0, j = 0;
	while(1)
	{
		for(; i <= flag; i++)
		{
			distance_count(&zuobiaodui[i][0], &zuobiaodui[i][1], &now_zuobiaodui[0], &now_zuobiaodui[1], &distance[i]);
			printf("到第i点的距离为%d\n", distance[i]);
		}//找到没走过的点之间的距离
	
	
		min_id	=	findMinExcept(distance, flag + 1, already_have, 100);
		//求出除了某几个点之外的最小的值的角标
		
		Set_Location_XOY(zuobiaodui[min_id][0] - now_zuobiaodui[0], zuobiaodui[min_id][1] - now_zuobiaodui[1], 1.5);
		Wait();


		now_zuobiaodui[0] = zuobiaodui[min_id][0];
		now_zuobiaodui[1] = zuobiaodui[min_id][1];			

		system_delay_ms(1500);
		
		
		ordered_zuobiaodui[j][0] = zuobiaodui[min_id][0];//赋值给按顺序走的坐标对
		ordered_zuobiaodui[j][1] = zuobiaodui[min_id][0];
		
		
		ordered_path[j] = min_id;//角标的路径
		already_have[j] = min_id;//已经走的角标
		printf("第%d次的角标是%d\n", min_id);
		if(j > flag)
		{
			printf("在j准备变为%d\n的时候退出", j+1);
			break;
		}
		j++;

	}

}

void distance_count(float *x1, float *y1, float *x2, float *y2, float *distance)
{
	float dis;
	*distance = sqrt((*x1 - *x2) * (*x1 - *x2) + (*y1 - *y2) * (*y1 - *y2));
}

int findMinExcept(float arr[], int n, int except[], int m) 
{
	int i, j, pos = 0;
	float min = arr[0];
	for (i = 0; i < n; i++) 
	{
		int flag = 0;
		for (j = 0; j < m; j++) 
		{
			if (i == except[j]) 
			{
					flag = 1;
					break;
			}
		}
		if (!flag && arr[i] < min)
		{
				min = arr[i];
				pos = i;
		}
	}
	return pos;
}
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


void Greedy_Algorithm(short int flag)//45�л���Щ����
{
	int i = 0, j = 0;
	while(1)
	{
		for(; i <= flag; i++)
		{
			distance_count(&zuobiaodui[i][0], &zuobiaodui[i][1], &now_zuobiaodui[0], &now_zuobiaodui[1], &distance[i]);
			printf("����i��ľ���Ϊ%d\n", distance[i]);
		}//�ҵ�û�߹��ĵ�֮��ľ���
	
	
		min_id	=	findMinExcept(distance, flag + 1, already_have, 100);
		//�������ĳ������֮�����С��ֵ�ĽǱ�
		
		Set_Location_XOY(zuobiaodui[min_id][0] - now_zuobiaodui[0], zuobiaodui[min_id][1] - now_zuobiaodui[1], 1.5);
		Wait();


		now_zuobiaodui[0] = zuobiaodui[min_id][0];
		now_zuobiaodui[1] = zuobiaodui[min_id][1];			

		system_delay_ms(1500);
		
		
		ordered_zuobiaodui[j][0] = zuobiaodui[min_id][0];//��ֵ����˳���ߵ������
		ordered_zuobiaodui[j][1] = zuobiaodui[min_id][0];
		
		
		ordered_path[j] = min_id;//�Ǳ��·��
		already_have[j] = min_id;//�Ѿ��ߵĽǱ�
		printf("��%d�εĽǱ���%d\n", min_id);
		if(j > flag)
		{
			printf("��j׼����Ϊ%d\n��ʱ���˳�", j+1);
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
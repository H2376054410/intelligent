#ifndef __GREEDY_ALGORITHM
#define __GREEDY_ALGORITHM

extern float* coordernation;//单个坐标点
extern float zuobiaodui[25][2];
extern float now_zuobiaodui[2];
extern int coordernation_num;
extern int flag;
extern int flag1;

extern int count;
extern float zoux;
extern float zouy;
extern union Test tep;

extern int min_id;
extern float min_distance;
extern int ordered_path[30];
extern float ordered_zuobiaodui[30][2];
//	float* distance = (float* )malloc(sizeof(float) * (flag + 1));
extern int already_have[100];
extern float distance[50];





extern void Greedy_Algorithm(short int flag);


#endif
#include "include.h"

float Current_X = 1, Current_Y = 1;
float Current_X_Temp = 1, Current_Y_Temp = 1, X_Temp = 0, Y_Temp = 0;
float Target_X, Target_Y;
float Rate_Temp, Rate_X_Y;
unsigned short int Flag_X_or_Y;

void Get_Location_2(void)
{
	if (Status != Status_Spin)
	{
		Current_X += (r / (2 * Correction * Correction_Straight)) * (Motor2_FL.Omega - Motor1_FR.Omega) * 0.05f;
		Current_Y += (r / (2 * Correction)) * (Motor2_FL.Omega + Motor1_FR.Omega) * 0.05f;
	}
}



void Get_Location(void)
{
	if (Flag_Straight_1 || Flag_Straight_2 || Flag_Straight_3)
	{
		if (Status == Status_13 && Flag_X_or_Y == 0)
		{
			Current_X = Current_X_Temp + Motor2_FL.Theta / Rate_Temp;
			Current_Y = Current_Y_Temp + Rate_X_Y * Motor2_FL.Theta / Rate_Temp;
		}
		else if (Status == Status_13 && Flag_X_or_Y == 1)
		{
			Current_Y = Current_Y_Temp + Motor2_FL.Theta / Rate_Temp;
			Current_X = Current_X_Temp + Rate_X_Y * Motor2_FL.Theta / Rate_Temp;
		}
		else if (Status == Status_24 && Flag_X_or_Y == 0)
		{
			Current_X = Current_X_Temp + Motor1_FR.Theta / Rate_Temp;
			Current_Y = Current_Y_Temp + Rate_X_Y * Motor1_FR.Theta / Rate_Temp;
		}
		else if (Status == Status_24 && Flag_X_or_Y == 1)
		{
			Current_Y = Current_Y_Temp + Motor1_FR.Theta / Rate_Temp;
			Current_X = Current_X_Temp + Rate_X_Y * Motor1_FR.Theta / Rate_Temp;	
		}
	}
	else
	{
		Current_X_Temp = Current_X;
		Current_Y_Temp = Current_Y;
	}	
}

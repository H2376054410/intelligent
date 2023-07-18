#ifndef __SWEEPING_SEARCH
#define __SWEEPING_SEARCH

#define Width_Of_View 0.4f

extern unsigned short int Flag_If_Recieved_Picture;
extern short int Times_Of_Yellow_Correction;



extern void Recognize_All_Dot(void);
extern void Place_Picture(void);
extern void Go_Back(void);
extern void Lets_Start(void);
extern void Tell_Art_To_Search_For_Yellow_Line(void);
extern void Interrupt_To_Pick_Picture(void);
extern void Decelerate(void);
extern void Dot_Correction(void);


#endif
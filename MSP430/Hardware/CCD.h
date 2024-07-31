#ifndef _CCD_H
#define _CCD_H
#include "board.h"
unsigned int adc_getValue(void);

void RD_TSL(void);
void  Find_CCD_Median (void); 

extern int Turn_Flag;


#endif
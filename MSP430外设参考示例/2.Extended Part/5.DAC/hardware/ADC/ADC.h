#ifndef __ADC_H_
#define __ADC_H_

#include "ti_msp_dl_config.h"

extern volatile uint16_t gAdcResult[1];

void ADC_Init(void);
void ADC_start(void);



#endif




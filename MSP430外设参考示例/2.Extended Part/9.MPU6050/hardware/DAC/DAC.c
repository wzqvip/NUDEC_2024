#include "..\HARDWARE\DAC\DAC.h"

#define COMP_0_INST_DAC8_OUTPUT_VOLTAGE_mV (1000)
#define COMP_0_INST_REF_VOLTAGE_mV (3300)
void DAC_Init(void)
{
    DL_COMP_setDACCode0(COMP_0_INST, 0);
    DL_COMP_enable(COMP_0_INST);
    DL_OPA_enable(OPA_0_INST);
}


void DAC_start(uint32_t DAC_value)
{
  
    DL_COMP_setDACCode0(COMP_0_INST, DAC_value);
}



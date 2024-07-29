#include "..\software\MYSPI\MYSPI.h"


void SPI_Send_cmd(uint8_t cmd)
{
	while (DL_SPI_isBusy(SPI_0_INST));
	DL_SPI_setControllerCommandDataModeConfig(SPI_0_INST, 1);
	while (DL_SPI_isBusy(SPI_0_INST));
	DL_SPI_transmitData8(SPI_0_INST, cmd);
}


uint8_t SPI_WriteByte(uint8_t Byte)
{
	while (DL_SPI_isBusy(SPI_0_INST));
	DL_SPI_transmitData8(SPI_0_INST, Byte);
	while(  DL_SPI_isRXFIFOEmpty(SPI_0_INST));
	//while(RESET == spi_i2s_flag_get(SPIx, SPI_FLAG_RBNE));
	return DL_SPI_receiveData8(SPI_0_INST);
}


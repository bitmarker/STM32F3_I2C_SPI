#include "L3GD20.h"



void L3GD20_Init(L3GD20_DATA *self)
{
  self->spi = 0;
  self->powerMode = 1;
  self->axisEnable = 0x07;
  self->dataRate = 0;
  self->bandWidth = 0;
}

void L3GD20_CleanUp(L3GD20_DATA *self)
{
  
}

void L3GD20_Idle(L3GD20_DATA *self)
{
  
}

void L3GD20_AssignSpi(L3GD20_DATA *self, EASYSPI_DATA *spi, uint8_t chipSelect)
{
  self->spi = spi;
  self->chipSelect = chipSelect;
}

void L3GD20_ReadData(L3GD20_DATA *self, uint8_t addr, uint8_t *data, uint8_t len)
{
  int i;
  uint8_t tempData[L3GD20_BUFFER_SIZE];
  
  /* Bit 0 is read bit */
  /* Bit 1 is multi bit */
  tempData[0] = (addr & 0x3F) | 0x80;
  
  /* Multibyte read */
  if(len > 1)
  {
    tempData[0] |= 0x40;
  }
  
  /* Write dummy data */
  for(i = 0; i < len; i++)
  {
    tempData[i + 1] = 0x00;
  }
  
  self->spi->transceive(self->spi, self->chipSelect, tempData, len + 1);
  
  for(i = 0; i < len; i++)
  {
    data[i] = tempData[i + 1];
  }
}

void L3GD20_WriteData(L3GD20_DATA *self, uint8_t addr, uint8_t *data, uint8_t len)
{
  int i;
  
  uint8_t tempData[L3GD20_BUFFER_SIZE];
  
  /* Bit 0 is read bit */
  /* Bit 1 is multi bit */
  tempData[0] = (addr & 0x3F);
  
  /* Multibyte read */
  if(len > 1)
  {
    tempData[0] |= 0x40;
  }
  
  /* Write data */
  for(i = 0; i < len; i++)
  {
    tempData[i + 1] = data[i];
  }
  
  self->spi->transceive(self->spi, self->chipSelect, tempData, len + 1);
}

RESULT L3GD20_Config(L3GD20_DATA *self)
{
  uint8_t reg1 = 0x00;
  uint8_t tmp = 0x00;
  
  volatile uint8_t temp;
  
  reg1 |= (self->dataRate & 0x03) << 7;
  reg1 |= (self->bandWidth & 0x03) << 5;
  reg1 |= (self->powerMode & 0x01) << 3;
  reg1 |= (self->axisEnable & 0x07);
  
  tmp = reg1;
  
  L3GD20_WriteData(self, L3GD20_CMD_CTRL_REG1, &reg1, 1);
  
  CALL_FUNC(self->wait_ms, 1);
  
  L3GD20_ReadData(self, L3GD20_CMD_CTRL_REG1, &reg1, 1);
  
  if(tmp != reg1)
  {
#ifdef DEBUG
    printf("tmp(%X) != reg1(%X)\n", tmp, reg1);
#endif
    return ERROR;
  }
  
  CALL_FUNC(self->wait_ms, 5);
  
  return SUCCESS;
}

uint8_t L3GD20_ReadStatus(L3GD20_DATA *self)
{
  uint8_t ret;
  
  L3GD20_ReadData(self, L3GD20_CMD_STATUS, &ret, 1);
  
  return ret;
}

RESULT L3GD20_CheckWhoIAm(L3GD20_DATA *self)
{
  uint8_t ret;
  
  L3GD20_ReadData(self, L3GD20_CMD_WHOAMI, &ret, 1);
  
  if(ret == L3GD20_RES_WHOAMI)
  {
    return SUCCESS;
  }
  
  return ERROR;
}
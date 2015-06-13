#ifndef __L3GD20_H__
#define __L3GD20_H__

#include <stdio.h>
#include "common.h"
#include "easy_spi.h"


typedef struct __l3gd20_public_data
{
  EASYSPI_DATA *spi;
  uint8_t chipSelect;
  uint8_t dataRate;
  uint8_t bandWidth;
  uint8_t powerMode;
  uint8_t axisEnable;
} L3GD20_DATA;



/* Private functions */


/* Public functions */
void L3GD20_Init(L3GD20_DATA *self);
void L3GD20_CleanUp(L3GD20_DATA *self);
void L3GD20_Idle(L3GD20_DATA *self);
void L3GD20_AssignSpi(L3GD20_DATA *self, EASYSPI_DATA *spi, uint8_t chipSelect);
void L3GD20_ReadData(L3GD20_DATA *self, uint8_t addr, uint8_t *data, uint8_t len);

void L3GD20_Config(L3GD20_DATA *self);

#endif

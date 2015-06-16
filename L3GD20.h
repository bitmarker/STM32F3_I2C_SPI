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
  WAIT_MS_FUNC wait_ms;
} L3GD20_DATA;

#define L3GD20_BUFFER_SIZE      8


#define L3GD20_CMD_WHOAMI       0x0F
#define L3GD20_RES_WHOAMI       0xD4

#define L3GD20_CMD_STATUS       0x27
#define L3GD20_CMD_CTRL_REG1    0x20


/* Private functions */


/* Public functions */
void L3GD20_Init(L3GD20_DATA *self);
void L3GD20_CleanUp(L3GD20_DATA *self);
void L3GD20_Idle(L3GD20_DATA *self);
void L3GD20_AssignSpi(L3GD20_DATA *self, EASYSPI_DATA *spi, uint8_t chipSelect);
void L3GD20_ReadData(L3GD20_DATA *self, uint8_t addr, uint8_t *data, uint8_t len);

RESULT L3GD20_Config(L3GD20_DATA *self);
uint8_t L3GD20_ReadStatus(L3GD20_DATA *self);
RESULT L3GD20_CheckWhoIAm(L3GD20_DATA *self);

#endif

#include "easy_spi.h"

void EASYSPI_Init(EASYSPI_DATA *self)
{
  self->isBusy = 0;
  self->rxCounter = 0;
  self->txCounter = 0;
}

void EASYSPI_SetUpChipSelects(EASYSPI_DATA *self)
{
  int i = 0;
  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* SPI Chip Select */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  for(i = 0; i < EASYSPI_MAX_CHIP_SELECTS; i++)
  {
    /* Enable CS periph */
    RCC_AHBPeriphClockCmd(self->pinCS[i].periph, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = self->pinCS[i].pin;
    
    GPIO_Init(self->pinCS[i].port, &GPIO_InitStructure);
    
    EASYSPI_ChipDeselect(self, i);
  }
}

void EASYSPI_SetDefaults(EASYSPI_DATA *self)
{
  /* Clock pin */
  self->pinCLK.pin = EASY_SPI_SPIx_SCK_PIN;
  self->pinCLK.port = EASY_SPI_SPIx_SCK_GPIO_PORT;
  self->pinCLK.periph = EASY_SPI_SPIx_SCK_GPIO_CLK;
  self->pinCLK.altFunc = EASY_SPI_SPIx_SCK_AF;
  self->pinCLK.source = EASY_SPI_SPIx_SCK_SOURCE;
  
  /* MISO pin */
  self->pinMISO.pin = EASY_SPI_SPIx_MISO_PIN;
  self->pinMISO.port = EASY_SPI_SPIx_MISO_GPIO_PORT;
  self->pinMISO.periph = EASY_SPI_SPIx_MISO_GPIO_CLK;
  self->pinMISO.altFunc = EASY_SPI_SPIx_MISO_AF;
  self->pinMISO.source = EASY_SPI_SPIx_MISO_SOURCE;
  
  /* MOSI pin */
  self->pinMOSI.pin = EASY_SPI_SPIx_MOSI_PIN;
  self->pinMOSI.port = EASY_SPI_SPIx_MOSI_GPIO_PORT;
  self->pinMOSI.periph = EASY_SPI_SPIx_MOSI_GPIO_CLK;
  self->pinMOSI.altFunc = EASY_SPI_SPIx_MOSI_AF;
  self->pinMOSI.source = EASY_SPI_SPIx_MOSI_SOURCE;
  
  /* CS pin [0] */
  self->pinCS[0].pin = EASY_SPI_SPIx_CS_GPIO_PORT_PIN;
  self->pinCS[0].port = EASY_SPI_SPIx_CS_GPIO_PORT;
  self->pinCS[0].periph = EASY_SPI_SPIx_CS_PERIPH;
  
  /* SPI Perip Clock */
  self->spi = EASY_SPI_SPIx;
  self->spiPeriph = EASY_SPI_SPIx_CLK;
}

void EASYSPI_Config(EASYSPI_DATA *self)
{
  EASYSPI_SetUpChipSelects(self);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(self->spiPeriph, ENABLE);
  
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(self->pinCLK.periph | self->pinMISO.periph | self->pinMOSI.periph, ENABLE);
  
  GPIO_PinAFConfig(self->pinCLK.port, self->pinCLK.source, self->pinCLK.altFunc);
  GPIO_PinAFConfig(self->pinMOSI.port, self->pinMOSI.source, self->pinMOSI.altFunc);
  GPIO_PinAFConfig(self->pinMISO.port, self->pinMISO.source, self->pinMISO.altFunc);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = self->pinCLK.pin;
  GPIO_Init(self->pinCLK.port, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  self->pinMOSI.pin;
  GPIO_Init(self->pinMOSI.port, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = self->pinMISO.pin;
  GPIO_Init(self->pinMISO.port, &GPIO_InitStructure);
      
  /* SPI configuration */
  SPI_I2S_DeInit(self->spi);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = EASYSPI_DATASIZE;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  
  /* Configure the Priority Group to 1 bit */                
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* Initializes the SPI communication */
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(self->spi, &SPI_InitStructure);
  
  /* Initialize the FIFO threshold */
  SPI_RxFIFOThresholdConfig(self->spi, SPI_RxFIFOThreshold_QF);
  
  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(self->spi, SPI_I2S_IT_RXNE, ENABLE);
  
  /* Enable the SPI Error interrupt */
  //SPI_I2S_ITConfig(self->spi, SPI_I2S_IT_ERR, ENABLE);
  
  /* Data transfer is performed in the SPI interrupt routine */
   
  /* Enable the SPI peripheral */
  SPI_Cmd(self->spi, ENABLE);
}

void EASYSPI_CleanUp(EASYSPI_DATA *self)
{
}

void EASYSPI_Idle(EASYSPI_DATA *self)
{
}

void EASYSPI_EnableTxInterrupt(EASYSPI_DATA *self)
{
  SPI_I2S_ITConfig(self->spi, SPI_I2S_IT_TXE, ENABLE);
}

void EASYSPI_DisableTxInterrupt(EASYSPI_DATA *self)
{
  SPI_I2S_ITConfig(self->spi, SPI_I2S_IT_TXE, DISABLE);
}

void EASYSPI_ChipSelect(EASYSPI_DATA *self, uint8_t index)
{
  GPIO_WriteBit(self->pinCS[index].port, self->pinCS[index].pin, Bit_RESET);
  self->lastChipSelect = index;
}
 
void EASYSPI_ChipDeselect(EASYSPI_DATA *self, uint8_t index)
{
  GPIO_WriteBit(self->pinCS[index].port, self->pinCS[index].pin, Bit_SET);
}

void EASYSPI_Write(EASYSPI_DATA *self, uint8_t chip_select, uint8_t *data, uint16_t len)
{
  int i;
  
  while(self->isBusy){}
  
  self->rxCounter = len;
  self->txCounter = len;
  
  self->isBusy = 1;
  
  for(i = 0; i < len; i++)
  {
    self->dataBuffer[len - 1 - i] = data[i];
  }
  
  tempData = self;
  
  EASYSPI_ChipSelect(self, chip_select);
  
  EASYSPI_EnableTxInterrupt(self);
  
  /* Waiting until TX FIFO is empty */
  while (SPI_GetTransmissionFIFOStatus(self->spi) != SPI_TransmissionFIFOStatus_Empty) {}
  
  /* Wait busy flag */
  while (SPI_I2S_GetFlagStatus(self->spi, SPI_I2S_FLAG_BSY) == SET) {}
  
  /* Waiting until RX FIFO is empty */
  while (SPI_GetReceptionFIFOStatus(self->spi) != SPI_ReceptionFIFOStatus_Empty) {}
  
  for(i = 0; i < len; i++)
  {
    data[i] = self->dataBuffer[len - 1 - i];
  }
}

void SPI1_IRQHandler(void)
{
  if(SPI_I2S_GetFlagStatus(tempData->spi, SPI_I2S_FLAG_RXNE) == SET)
  {
    // Receive Buffer Not Empty
    tempData->rxCounter--;
    tempData->dataBuffer[tempData->rxCounter] = SPI_ReceiveData8(tempData->spi);
 
    if(tempData->rxCounter == 0)
    {
      EASYSPI_ChipDeselect(tempData, tempData->lastChipSelect);
      tempData->isBusy = 0;
    }
  }
  else if(SPI_I2S_GetFlagStatus(tempData->spi, SPI_I2S_FLAG_TXE) == SET)
  {
    // Transmit Buffer Empty
    if(tempData->txCounter > 0)
    {
      SPI_SendData8(tempData->spi, tempData->dataBuffer[tempData->txCounter - 1]);
      tempData->txCounter--;
    }
    else
    {
      EASYSPI_DisableTxInterrupt(tempData);
    }
  }
}
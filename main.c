#include <stdio.h>
#include "common.h"

#include "easy_spi.h"

#define BUTTON_GPIO             GPIOA
#define BUTTON_GPIO_PERIPH      RCC_AHBPeriph_GPIOA
#define BUTTON_PIN              GPIO_Pin_0
#define BUTTON_EXTI_SRC         EXTI_PortSourceGPIOA
#define BUTTON_EXTI_SRC_PIN     EXTI_PinSource0
#define BUTTON_EXTI_LINE        EXTI_Line0

void EXTI0_Config()
{
  /* Init the GPIO */
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
    
  RCC_AHBPeriphClockCmd(BUTTON_GPIO_PERIPH, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN;
  GPIO_Init(BUTTON_GPIO, &GPIO_InitStructure);

  
  /* Init the Interrupt source */
  
  SYSCFG_EXTILineConfig(BUTTON_EXTI_SRC, BUTTON_EXTI_SRC_PIN);

  EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}




//#define SPI_DATASIZE SPI_DataSize_8b
//
///* Communication boards SPIx Interface */
//#define SPIx                             SPI1
//#define SPIx_CLK                         RCC_APB2Periph_SPI1
//#define SPIx_IRQn                        SPI1_IRQn
//#define SPIx_IRQ_HANDLER                 SPI1_IRQHandler
//
//#define SPI_PORT                         GPIOA
//#define SPI_PERIPH                       RCC_AHBPeriph_GPIOA
//
//#define SPIx_SCK_PIN                     GPIO_Pin_5
//#define SPIx_SCK_GPIO_PORT               SPI_PORT
//#define SPIx_SCK_GPIO_CLK                SPI_PERIPH
//#define SPIx_SCK_SOURCE                  GPIO_PinSource5
//#define SPIx_SCK_AF                      GPIO_AF_5
//
//#define SPIx_MISO_PIN                    GPIO_Pin_6
//#define SPIx_MISO_GPIO_PORT              SPI_PORT
//#define SPIx_MISO_GPIO_CLK               SPI_PERIPH
//#define SPIx_MISO_SOURCE                 GPIO_PinSource6
//#define SPIx_MISO_AF                     GPIO_AF_5
//
//#define SPIx_MOSI_PIN                    GPIO_Pin_7
//#define SPIx_MOSI_GPIO_PORT              SPI_PORT
//#define SPIx_MOSI_GPIO_CLK               SPI_PERIPH
//#define SPIx_MOSI_SOURCE                 GPIO_PinSource7
//#define SPIx_MOSI_AF                     GPIO_AF_5
//
//#define SPIx_CS_GPIO_PORT                GPIOE
//#define SPIx_CS_GPIO_PORT_Pin            GPIO_Pin_3
//#define SPIx_CS_PERIPH                   RCC_AHBPeriph_GPIOE
//
//SPI_InitTypeDef  SPI_InitStructure;
//
//static void SPI_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  /* Enable the SPI periph */
//  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);
//  
//  /* Enable CS periph */
//  RCC_AHBPeriphClockCmd(SPIx_CS_PERIPH, ENABLE);
//  
//  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
//  RCC_AHBPeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);
//  
//  GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
//  GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
//  GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
//  
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//
//  /* SPI SCK pin configuration */
//  GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
//  GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);
//
//  /* SPI  MOSI pin configuration */
//  GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
//  GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
//
//  /* SPI MISO pin configuration */
//  GPIO_InitStructure.GPIO_Pin = SPIx_MISO_PIN;
//  GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
//  
//  
//  /* SPI Chip Select */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Pin = SPIx_CS_GPIO_PORT_Pin;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;;
//  GPIO_Init(SPIx_CS_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_WriteBit(SPIx_CS_GPIO_PORT, SPIx_CS_GPIO_PORT_Pin, Bit_SET);
//  
//    
//  /* SPI configuration -------------------------------------------------------*/
//  SPI_I2S_DeInit(SPIx);
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_DataSize = SPI_DATASIZE;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//  SPI_InitStructure.SPI_CRCPolynomial = 7;
//  
//  /* Configure the Priority Group to 1 bit */                
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  
//  /* Configure the SPI interrupt priority */
//  NVIC_InitStructure.NVIC_IRQChannel = SPIx_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  
//  
//  /* Initializes the SPI communication */
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_Init(SPIx, &SPI_InitStructure);
//  
//  /* Initialize the FIFO threshold */
//  SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
//  
//  /* Enable the Rx buffer not empty interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
//  
//  /* Enable the SPI Error interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
//  
//  /* Data transfer is performed in the SPI interrupt routine */
//   
//  /* Enable the SPI peripheral */
//  SPI_Cmd(SPIx, ENABLE);
//}

void SysTick_Handler(void)
{
    
}

//#define BUFFER_SIZE 2
//uint8_t spiRxCounter;
//uint8_t spiTxCounter;
//uint8_t spiBusyFlag;
//uint8_t spiDataBuffer[BUFFER_SIZE];


//void SPI_EnableTxInterrupt(void)
//{
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);
//}
// 
//void SPI_DisableTxInterrupt(void)
//{
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);
//}
// 
//void SPI_ChipSelect(void)
//{
//  GPIO_WriteBit(SPIx_CS_GPIO_PORT, SPIx_CS_GPIO_PORT_Pin, Bit_RESET);
//}
// 
//void SPI_ChipDeselect(void)
//{
//  GPIO_WriteBit(SPIx_CS_GPIO_PORT, SPIx_CS_GPIO_PORT_Pin, Bit_SET);
//}

//void SPI_WriteTwoBytes(uint8_t byte1, uint8_t byte0)
//{
//  while(spiBusyFlag){}
//  spiTxCounter = 2;
//  spiRxCounter = 2;
//  spiBusyFlag = 1;
//  
//  spiDataBuffer[0] = byte0;
//  spiDataBuffer[1] = byte1;
//
//  
//  SPI_ChipSelect();
//  
//  SPI_EnableTxInterrupt();
//  
///* Waiting until TX FIFO is empty */
//  while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty) {}
//  
//  /* Wait busy flag */
//  while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET) {}
//  
//  /* Waiting until RX FIFO is empty */
//  while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty) {}
//}

int write_spi = 0;

void EXTI0_IRQHandler(void)
{

  
  if((EXTI_GetITStatus(BUTTON_EXTI_LINE) != RESET))
  {
    write_spi = 1;      
  }
  
  EXTI_ClearITPendingBit(BUTTON_EXTI_LINE);
}

int main(void)
{
  __IO uint16_t byte0, byte1;
  
  
  uint8_t data[2];
  
  EXTI0_Config();
  
  EASYSPI_DATA spi_data;
  
  EASYSPI_Init(&spi_data);
  EASYSPI_SetDefaults(&spi_data);
  EASYSPI_Config(&spi_data);
  
  write_spi = 1;
  
  
  /* Infinite loop */
  while (1)
  {
    if(write_spi)
    {
      write_spi = 0;
      
      // to Slave:    0x8F 0x00
      // from Slave:  0x00 0xD4
      
      data[0] = 0x8F;
      data[1] = 0;
      
      EASYSPI_Write(&spi_data, 0, data, 2);
      
      byte0 = data[0];
      byte1 = data[1];
      
      asm("nop");
      
      
    }
  }
}




//void SPIx_IRQ_HANDLER(void)
//{
//  if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == SET)
//  {
//    // Receive Buffer Not Empty
//    spiRxCounter--;
//    spiDataBuffer[spiRxCounter] = SPI_ReceiveData8(SPIx);
// 
//    if(spiRxCounter == 0)
//    {
//      SPI_ChipDeselect();
//      spiBusyFlag = 0;
//    }
//  }
//  else if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == SET)
//  {
//    // Transmit Buffer Empty
//    if(spiTxCounter != 0)
//    {
//      SPI_SendData8(SPIx, spiDataBuffer[spiTxCounter - 1]);
//      spiTxCounter--;
//    }
//    else
//    {
//      SPI_DisableTxInterrupt();
//    }
//  }
//}
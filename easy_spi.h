#ifndef __EASYSPI_H
#define __EASYSPI_H

#include <stdio.h>
#include "common.h"

#ifndef EASYSPI_BUFFER_SIZE
  #define EASYSPI_BUFFER_SIZE 8
#endif

#ifndef EASYSPI_MAX_CHIP_SELECTS
  #define EASYSPI_MAX_CHIP_SELECTS 1
#endif

#ifndef EASYSPI_DATASIZE
  #define EASYSPI_DATASIZE SPI_DataSize_8b
#endif

/* Default Settings */
#ifndef EASY_SPI_DEFAULTS
  #define EASY_SPI_SPIx                             SPI1
  #define EASY_SPI_SPIx_CLK                         RCC_APB2Periph_SPI1
  #define EASY_SPI_SPIx_IRQn                        SPI1_IRQn

  #define EASY_SPI_SPI_PORT                         GPIOA
  #define EASY_SPI_SPI_PERIPH                       RCC_AHBPeriph_GPIOA
   
  #define EASY_SPI_SPIx_SCK_PIN                     GPIO_Pin_5
  #define EASY_SPI_SPIx_SCK_GPIO_PORT               EASY_SPI_SPI_PORT
  #define EASY_SPI_SPIx_SCK_GPIO_CLK                EASY_SPI_SPI_PERIPH
  #define EASY_SPI_SPIx_SCK_SOURCE                  GPIO_PinSource5
  #define EASY_SPI_SPIx_SCK_AF                      GPIO_AF_5

  #define EASY_SPI_SPIx_MISO_PIN                    GPIO_Pin_6
  #define EASY_SPI_SPIx_MISO_GPIO_PORT              EASY_SPI_SPI_PORT
  #define EASY_SPI_SPIx_MISO_GPIO_CLK               EASY_SPI_SPI_PERIPH
  #define EASY_SPI_SPIx_MISO_SOURCE                 GPIO_PinSource6
  #define EASY_SPI_SPIx_MISO_AF                     GPIO_AF_5

  #define EASY_SPI_SPIx_MOSI_PIN                    GPIO_Pin_7
  #define EASY_SPI_SPIx_MOSI_GPIO_PORT              EASY_SPI_SPI_PORT
  #define EASY_SPI_SPIx_MOSI_GPIO_CLK               EASY_SPI_SPI_PERIPH
  #define EASY_SPI_SPIx_MOSI_SOURCE                 GPIO_PinSource7
  #define EASY_SPI_SPIx_MOSI_AF                     GPIO_AF_5

  #define EASY_SPI_SPIx_CS_GPIO_PORT                GPIOE
  #define EASY_SPI_SPIx_CS_GPIO_PORT_PIN            GPIO_Pin_3
  #define EASY_SPI_SPIx_CS_PERIPH                   RCC_AHBPeriph_GPIOE
  
  #define EASY_SPI_DEFAULTS
#endif


typedef struct __easyspi_pin_data
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint32_t periph;
  uint8_t source;
  uint8_t altFunc;
} EASYSPI_PIN_DATA;

typedef struct __easyspi_public_data
{
  volatile uint8_t rxCounter;
  volatile uint8_t txCounter;
  volatile uint8_t isBusy;
  uint8_t dataBuffer[EASYSPI_BUFFER_SIZE];
  SPI_TypeDef *spi;
  uint32_t spiPeriph;
  EASYSPI_PIN_DATA pinMOSI;
  EASYSPI_PIN_DATA pinMISO;
  EASYSPI_PIN_DATA pinCLK;
  EASYSPI_PIN_DATA pinCS[EASYSPI_MAX_CHIP_SELECTS];
  uint8_t lastChipSelect;
} EASYSPI_DATA;


/* Private functions */
static void EASYSPI_EnableTxInterrupt();
static void EASYSPI_DisableTxInterrupt();

/* Public functions */
void EASYSPI_Init(EASYSPI_DATA *self);
void EASYSPI_CleanUp(EASYSPI_DATA *self);
void EASYSPI_Idle(EASYSPI_DATA *self);
void EASYSPI_SetDefaults(EASYSPI_DATA *self);
void EASYSPI_Config(EASYSPI_DATA *self);
void EASYSPI_ChipSelect(EASYSPI_DATA *self, uint8_t index);
void EASYSPI_ChipDeselect(EASYSPI_DATA *self, uint8_t index);

void EASYSPI_Transceive(EASYSPI_DATA *self, uint8_t chip_select, uint8_t *data, uint16_t len);

#endif
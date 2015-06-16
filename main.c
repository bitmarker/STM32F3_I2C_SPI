#include <stdio.h>
#include "common.h"

#include "easy_spi.h"
#include "L3GD20.h"


volatile uint32_t ticks = 0;




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





void SysTick_Handler(void)
{
    ticks++;
}



int write_spi = 0;

void EXTI0_IRQHandler(void)
{

  
  if((EXTI_GetITStatus(BUTTON_EXTI_LINE) != RESET))
  {
    write_spi = 1;      
  }
  
  EXTI_ClearITPendingBit(BUTTON_EXTI_LINE);
}

void wait_ms_callback(uint16_t count)
{
  uint32_t t = ticks;
  while(ticks - t < count) {}
}

int main(void)
{
  __IO uint8_t byte0, byte1;
  uint8_t data[2];
  
  EXTI0_Config();
  
  /* 1ms */
  SysTick_Config(SystemCoreClock / 1000);
  
  EASYSPI_DATA spi_data;
  L3GD20_DATA l3gd20_data;
  
  EASYSPI_Init(&spi_data);
  EASYSPI_SetDefaults(&spi_data);
  EASYSPI_Config(&spi_data);
  
  
  L3GD20_Init(&l3gd20_data);
  L3GD20_AssignSpi(&l3gd20_data, &spi_data, 0);
  l3gd20_data.wait_ms = wait_ms_callback;
    
  if(L3GD20_CheckWhoIAm(&l3gd20_data) == ERROR)
  {
    EXIT("L3GD20 not found!");
  }
  
  wait_ms_callback(10);
  
  if(L3GD20_Config(&l3gd20_data) == ERROR)
  {
    EXIT("L3GD20 not ready!");
  }
  
  write_spi = 1;
  
  TRACE("Start!");
  
  /* Infinite loop */
  while (1)
  {
    if(write_spi)
    {
      write_spi = 0;
      
      
      // to Slave:    0x8F 0x00
      // from Slave:  0x00 0xD4
      
      
      /*  */
      //byte0 = L3GD20_ReadStatus(&l3gd20_data);
      
      
      L3GD20_ReadData(&l3gd20_data, 0x26, &data[0], 1);
      
      
      printf("%X\n", data[0]);
      
      asm("nop");
      
      
      
    }
  }
}

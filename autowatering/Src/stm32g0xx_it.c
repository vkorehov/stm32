/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;

/* During the cleanup phase in EE_Init, AddressRead is the address being read */ 
extern volatile uint32_t AddressRead;
/* Flag equal to 1 when the cleanup phase is in progress, 0 if not */
extern volatile uint8_t CleanupPhase;

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* Check if NMI is due to flash ECCD (error detection) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
  {
    if(CleanupPhase==1)
    {
      if ((AddressRead >= START_PAGE_ADDRESS) && (AddressRead <= END_EEPROM_ADDRESS))
      {
        /* Delete the corrupted flash address */
        if (EE_DeleteCorruptedFlashAddress((uint32_t)AddressRead) == EE_OK)
        {
          /* Resume execution if deletion succeeds */
          return;
        }
        /* If we do not succeed to delete the corrupted flash address */
        /* This might be because we try to write 0 at a line already considered at 0 which is a forbidden operation */
        /* This problem triggers PROGERR, PGAERR and PGSERR flags */
        else
        {
          /* We check if the flags concerned have been triggered */
          if((__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR)) && (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR))  
             && (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR)))
          {
            /* If yes, we clear them */
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PROGERR);
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
            
            /* And we exit from NMI without doing anything */
            /* We do not invalidate that line because it is not programmable at 0 till the next page erase */
            /* The only consequence is that this line will trigger a new NMI later */
            return;
          }
        }
      }
    }
    else
    {
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);
      return;
    }
  }
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}
/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

void OnTick(void);
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  OnTick();
}


/**
  * @brief  This function handles Flash interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_IRQHandler(void)
{
  if( (FLASH->ECCR & FLASH_FLAG_ECCC) != 0 ){
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCC);
  }
  HAL_FLASH_IRQHandler();
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void IDDLE_UART_IRQHandler(UART_HandleTypeDef *huart);
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
  IDDLE_UART_IRQHandler(&huart1); // IDLE detection
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

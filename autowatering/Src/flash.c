#include "flash.h"

void MX_FLASH_Init(void)
{
  EE_Status ee_status = EE_OK;
  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);
  
  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); 
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);
  
  /* Clear the Standby flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  
  /* Check and Clear the Wakeup flag */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != RESET)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
  }
  /* System reset comes from a STANDBY wakeup: Conditional Erase*/
  /* Initialize EEPROM emulation driver (mandatory) */
  ee_status = EE_Init(EE_CONDITIONAL_ERASE);
  if(ee_status != EE_OK) {Error_Handler();}
}
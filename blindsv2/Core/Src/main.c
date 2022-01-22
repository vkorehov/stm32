/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "flash.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

void SendOK(uint32_t status);
void SendErrorStatus(void);
extern const uint8_t crc8_table[];
const uint8_t myAddress = 0x15;

#define MAX_DURATION 65000

uint8_t hal_err = 0;
uint8_t move_status = 0;
uint8_t invert = 0;
uint8_t lowered = 1;

uint16_t duration = 0;

uint8_t uart_state = 0;
char uart_rx_buffer[32] = {0xff};
char uart_tx_buffer[256] = {0};
uint8_t send_debug_on = 0;
uint8_t send_invert_on = 0;
uint8_t send_invert_off = 0;
uint8_t send_forced_debug_on = 0;
uint8_t send_debug_off = 0;
uint8_t send_error_status = 0;
uint8_t send_reset = 0;
uint8_t send_duration_set = 0;
uint8_t send_move_up = 0;
uint8_t send_move_up_ok = 0;
uint8_t send_move_down = 0;
uint8_t send_move_down_ok = 0;
uint16_t stop_move_up_in = 0;
uint16_t stop_move_down_in = 0;
uint8_t send_move_status = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void SaveVar(uint32_t idx, uint32_t val) {
  uint32_t flashVerify = 0;
  EE_Status ee_status = EE_OK;
  ee_status = EE_WriteVariable32bits(idx, val);
  ee_status|= EE_ReadVariable32bits(idx, &flashVerify);
  if (val != flashVerify) {
    Error_Handler();
  }  
  /* Start cleanup IT mode, if cleanup is needed */
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {
    ee_status|= EE_CleanUp();
  }
  if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {
    Error_Handler();
  }
}

uint32_t ReadVar(uint32_t idx) {
  uint32_t var = 0;
  EE_ReadVariable32bits(idx, &var);
  return var;
}

void ReceiveNextBuffer() {
  memset(uart_rx_buffer, 0xff, sizeof(uart_rx_buffer));
  while(1) {
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
    if (status == HAL_BUSY) {
      continue;
    }
    if (status != HAL_OK) {
      hal_err++;
      HAL_UART_Abort(&huart1);
      continue;
    }
    break;
  }  
}

void SendBuffer(char* buff, size_t sz) {
  if (uart_state == 1) { // only tx in valid state
    while(1) {
      HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, sz);
      if (status == HAL_BUSY) {
        continue;
      }
      if (status != HAL_OK) {
        hal_err++;
        HAL_UART_Abort(&huart1);
        continue;
      }
      break;
    }
  }
}

void CW(void) {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  
}

void CCW(void) {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  
}

void OFF(void) {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  
}

uint8_t SafeReadSensor(void) {
  static uint8_t accum = 0;  
  uint8_t r = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13);
  if (r == 0) { // active LOW
    accum <<= 1;
    accum |= 1;
  } else {
    accum <<= 1;
    accum &= ~1;
  }
  if (accum == 0xFF) {
    accum = 0;
    return 1;
  }
  return 0;
}

void StopMove(void) {
  OFF();
  move_status = 0;
}

void StartMove(uint8_t direction) {
  if (direction == 0x02) { // down
    if (lowered == 1) {
      // don't lower again!
      return;
    }
    lowered = 1;
    if (invert) {
      CCW();
    } else {
      CW();      
    }
    if (duration > 0) {
      stop_move_down_in = duration; // send last status in xxx ms      
    } else {
      stop_move_down_in = 1000; // send last status in 1000 ms            
    }    
    move_status = 1;    
  } else if (direction == 0x01) { // up
    lowered = 0;
    if (invert) {
      CW();
    } else {
      CCW();
    }
    stop_move_up_in = MAX_DURATION; // send last status in MAX_DURATION ms              
    move_status = 1;
  }
}

void OnTick(void) {
  
  if (stop_move_down_in > 0) {
    stop_move_down_in--;
    if (stop_move_down_in == 0) {
      StopMove();
    }
  }
  if (stop_move_up_in > 0) {
    stop_move_up_in--;
    
    if (stop_move_up_in == 0 || SafeReadSensor() == 1) {
      stop_move_up_in = 0;
      StopMove();
    }
  }
  
  if (send_duration_set > 0) {
    send_duration_set--;
    if (send_duration_set == 0) {
      SaveVar(2, duration);      
      SendOK(duration);
    }
  }
  
  if (send_debug_on > 0) {
    send_debug_on--;
    if (send_debug_on == 0) {
      SaveVar(1, 0x00);
      SendOK(0x00);
      send_reset = 200;      
    }
  }
  
  if (send_invert_on > 0) {
    send_invert_on--;
    if (send_invert_on == 0) {
      SaveVar(3, 0x01);
      invert = 0x01;      
      SendOK(0x01);
    }
  }
  if (send_invert_off > 0) {
    send_invert_off--;
    if (send_invert_off == 0) {
      SaveVar(3, 0x00);
      invert = 0x00;      
      SendOK(0x00);
    }
  }
  
  if (send_forced_debug_on > 0) {
    send_forced_debug_on--;
    if (send_forced_debug_on == 0) {
      EE_Status ee_status = EE_Init(EE_FORCED_ERASE);
      if(ee_status != EE_OK) {Error_Handler();}    
      SaveVar(1, 0x00);      
      SendOK(ee_status);
      send_reset = 200;      
    }
  }
  
  if (send_debug_off > 0) {
    send_debug_off--;
    if (send_debug_off == 0) {
      SaveVar(1, 0x01);
      SendOK(0x01);
      send_reset = 200;
    }
  }  
  
  if (send_error_status > 0) {
    send_error_status--;
    if (send_error_status == 0) {
      SendErrorStatus();
    }
  }  
    
  if (send_reset > 0) {
    send_reset--;
    if (send_reset == 0) {
      __NVIC_SystemReset();// GPIO/SWD switch doesnt work without reset            
    }
  }
  
  if (send_move_status > 0) {
    send_move_status--;
    if (send_move_status == 0) {
      SendOK(move_status);
    }
  }    
  if (send_move_up_ok > 0) {
    send_move_up_ok--;
    if (send_move_up_ok == 0) {
      SendOK(move_status);
    }
  }  
  if (send_move_down_ok > 0) {
    send_move_down_ok--;
    if (send_move_down_ok == 0) {
      SendOK(move_status);
    }
  }    
}


void OnCommand(void) {
  uint8_t crc = 0;
  uint8_t crcWordArg = 0;
  crc = crc8_table[crc ^ myAddress];
  crcWordArg = crc8_table[crc ^ uart_rx_buffer[1]];
  crcWordArg = crc8_table[crcWordArg ^ uart_rx_buffer[2]];
  crcWordArg = crc8_table[crcWordArg ^ uart_rx_buffer[3]];
  
  if (uart_rx_buffer[0] == myAddress && 
      uart_rx_buffer[1] == 0x01 &&
        uart_rx_buffer[2] == crc8_table[crc ^ 0x01])
  {
    uart_state = 1; // tx state
    send_move_status = 50; // send last status in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x02 &&
               uart_rx_buffer[4] == crcWordArg)
  {
    uart_state = 1; // tx state
    duration = ((uint16_t)uart_rx_buffer[2]) << 8 | uart_rx_buffer[3];
    send_duration_set = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x03 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x03])
  {
    uart_state = 1; // tx state
    send_invert_on = 50; // send last status in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x04 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x04])
  {
    uart_state = 1; // tx state
    send_invert_off = 50; // send last status in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x05 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x05])
  {
    uart_state = 1; // tx state
    send_move_down = 1; // in main loop immediately
    send_move_down_ok = 50; // send last status in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x06 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x06])
  {
    uart_state = 1; // tx state
    send_move_up = 1; // in main loop immediately
    send_move_up_ok = 50; // send last status in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x77 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x77])
  {
    uart_state = 1; // tx state
    send_debug_on = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x78 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x78])
  {
    uart_state = 1; // tx state
    send_forced_debug_on = 50; // in 50ms    
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x88 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x88])
  {
    uart_state = 1; // tx state
    send_debug_off = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x99 &&
               uart_rx_buffer[2] == crc8_table[crc ^ 0x99])
  {
    uart_state = 1; // tx state
    send_error_status = 50; // in 50ms
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_state == 1) {
    uart_state = 0;
    ReceiveNextBuffer();
  }
}

//HAL_UART_RxCpltCallback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {  
  if (uart_state == 0) {
    OnCommand();
  }
  if (uart_state == 0) {  
    ReceiveNextBuffer();    
  }
}

uint32_t uart_error = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_error = huart->ErrorCode;  
  if ((uart_error & HAL_UART_ERROR_PE) != 0U) {
    __HAL_UART_CLEAR_PEFLAG(&huart1);
  }  
  if ((uart_error & HAL_UART_ERROR_FE) != 0U) {
    __HAL_UART_CLEAR_FEFLAG(&huart1);
  }
  if ((uart_error & HAL_UART_ERROR_NE) != 0U) {
    __HAL_UART_CLEAR_NEFLAG(&huart1);
  }  
  // Restart reception ONLY in case Blocking error occurred  
  if ((uart_error & (HAL_UART_ERROR_RTO | HAL_UART_ERROR_ORE)) != 0U) {
    HAL_UART_Abort(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    
    // abort all tickers which result in uart tx
    send_duration_set=0;
    send_debug_on=0;
    send_invert_on=0;
    send_invert_off=0;
    send_forced_debug_on=0;
    send_debug_off=0;
    send_error_status=0;
    send_move_status=0;
    send_move_up_ok=0;
    send_move_down_ok = 0;
  
    uart_state = 0; // reset state      
    ReceiveNextBuffer();
  }
}

void SendErrorStatus(void) {
  int size = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "UART=%"PRIu32"|HAL=%"PRIu8"\n", uart_error, hal_err);
  if (size > 0) {
    SendBuffer(uart_tx_buffer, size); 
    uart_error = 0;
  }
}

void SendOK(uint32_t status) {
  int size = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "OK status=%"PRIu32"\n", status);
  if (size > 0) {
    SendBuffer(uart_tx_buffer, size);    
  }
}

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_FLASH_Init();
  MX_GPIO_Init();
  MX_USART1_UART_Init();  
  uint8_t no_debug = ReadVar(1);
  if (no_debug == 1) {
    MX_GPIO_NoDebug();    
  }
  HAL_SYSTICK_Config(16000U); // 1ms
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);   
  
  ReceiveNextBuffer();
  
  duration = ReadVar(2);
  invert = ReadVar(3);
  uint8_t deferred_move_type = 0;  
  /* Infinite loop */
  while (1)
  {
    if (send_move_up > 0) {
      send_move_up = 0;
      if (move_status > 0) {
        deferred_move_type = 0x01;        
      } else { // move immediately
        StartMove(0x01);
      }
    }
    if (send_move_down > 0) {
      send_move_down = 0;
      if (move_status > 0) {
        deferred_move_type = 0x02;
      } else { // move immediately
        StartMove(0x02);
      }      
    }
    // move deferred type AFTER movement stopped. never interrupt movement
    if (deferred_move_type > 0 && move_status == 0) {
      uint8_t t = deferred_move_type;
      deferred_move_type = 0;
      StartMove(t);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

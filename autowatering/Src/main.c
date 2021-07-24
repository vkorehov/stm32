/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
void SystemClock_Config(void);
void SendADCAverages(void);
void SendOK(uint32_t);

const uint8_t MAP_SIZE = 15;
GPIO_TypeDef* const PORT_MAP[] = {
  GPIOC,
  GPIOC,
  GPIOB,
  GPIOC,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOB,
  GPIOD,
  GPIOD,
  GPIOD,
  GPIOD,
  GPIOA
};
const uint16_t PIN_MAP[] = {
  GPIO_PIN_15,
  GPIO_PIN_14,
  GPIO_PIN_13,
  GPIO_PIN_13,
  GPIO_PIN_9,
  GPIO_PIN_8,
  GPIO_PIN_7,
  GPIO_PIN_6,
  GPIO_PIN_5,
  GPIO_PIN_4,
  GPIO_PIN_3,
  GPIO_PIN_2,
  GPIO_PIN_1,
  GPIO_PIN_0,
  GPIO_PIN_15
};

#define ADC_MAX_VAL 16400
// DMA readings of 15 inputs (last is 3x wires together, and before last one is 2x wires together)
volatile uint16_t ADC_BUFFER[] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,
                         0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
#define N_SHIFT (8)
uint32_t ADC_AVERAGE[] = {0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0};

/*
MA*[i]= MA*[i-1] +X[i] - MA*[i-1]/N
where MA* is the moving average*N.
MA[i]= MA*[i]/N
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  for(uint8_t i = 0; i < 15; i++) {
    ADC_AVERAGE[i] = ((ADC_AVERAGE[i] << N_SHIFT) - ADC_AVERAGE[i] + ((uint32_t)(ADC_BUFFER[i]))) >> N_SHIFT;
  }  
}

extern const uint8_t crc8_table[];
extern const uint8_t myAddress;
uint16_t treshold = 0;
uint16_t safe_treshold = 0;
uint16_t duration = 0;
uint8_t watering_status = 0;

uint8_t uart_state = 0;
char uart_rx_buffer[16] = {0xff};
char uart_tx_buffer[256] = {0};
uint8_t send_averages = 0;
uint8_t send_debug_on = 0;
uint8_t send_forced_debug_on = 0;
uint8_t send_debug_off = 0;
uint8_t send_reset = 0;
uint8_t send_treshold_set = 0;
uint8_t send_treshold_safe_set = 0;
uint8_t send_duration_set = 0;
uint8_t send_watering = 0;
uint8_t send_watering_ok = 0;

void ReceiveNextBuffer(void) {
    HAL_UART_Abort(&huart1);
    memset(uart_rx_buffer, 0xff, sizeof(uart_rx_buffer));
    while(1) {
      HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
      if (status == HAL_BUSY) {
        continue;
      }
      if (status != HAL_OK) {
        Error_Handler();
      }
      break;
    }  
}

void SendBuffer(char* buff, size_t sz) {
    HAL_UART_Abort(&huart1);
    while(1) {
      HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, sz);
      if (status == HAL_BUSY) {
        continue;
      }
      if (status != HAL_OK) {
        Error_Handler();
      }
      break;
    }  
}

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

uint8_t Watering(void) {
  uint8_t n = 0;
  for(uint8_t i = 0; i < MAP_SIZE; i++) {
    if (ADC_AVERAGE[i] >= safe_treshold) {
      HAL_Delay(50); // small delay for Rpi uart to work    
      return 0x66; // Abort wayrting and send special state
    }
  }
  for(uint8_t i = 0; i < MAP_SIZE; i++) {
    if (ADC_AVERAGE[i] < treshold) {
      n++;
      HAL_GPIO_WritePin(PORT_MAP[i], PIN_MAP[i], GPIO_PIN_SET);
      HAL_Delay(duration);
      HAL_GPIO_WritePin(PORT_MAP[i], PIN_MAP[i], GPIO_PIN_RESET);
      HAL_Delay(100); // small delay between pots
    }
  }
  return n;
}

void OnTick(void) {
  if (send_averages > 0) {
    send_averages--;
    if (send_averages == 0) {
      SendADCAverages();
    }
  }

  if (send_treshold_set > 0) {
    send_treshold_set--;
    if (send_treshold_set == 0) {
       SaveVar(2, treshold);      
       SendOK(treshold);
    }
  }
  
  if (send_treshold_safe_set > 0) {
    send_treshold_safe_set--;
    if (send_treshold_safe_set == 0) {
       SaveVar(3, safe_treshold);
       SendOK(safe_treshold);
    }
  }
  
  if (send_duration_set > 0) {
    send_duration_set--;
    if (send_duration_set == 0) {
       SaveVar(4, duration);
       SendOK(duration);
    }
  }  
  
  if (send_watering_ok > 0) {
    send_watering_ok--;
    if (send_watering_ok == 0) {
      SendOK(watering_status);
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
      
  if (send_reset > 0) {
    send_reset--;
    if (send_reset == 0) {
      __NVIC_SystemReset();// GPIO/SWD switch doesnt work without reset            
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
    send_averages = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x02 &&
             uart_rx_buffer[4] == crcWordArg)
  {
    uart_state = 1; // tx state
    treshold = ((uint16_t)uart_rx_buffer[2]) << 8 | uart_rx_buffer[3];
    send_treshold_set = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x03 &&
             uart_rx_buffer[4] == crcWordArg)
  {
    uart_state = 1; // tx state
    safe_treshold = ((uint16_t)uart_rx_buffer[2]) << 8 | uart_rx_buffer[3];
    send_treshold_safe_set = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x04 &&
             uart_rx_buffer[4] == crcWordArg)
  {
    uart_state = 1; // tx state
    duration = ((uint16_t)uart_rx_buffer[2]) << 8 | uart_rx_buffer[3];
    send_duration_set = 50; // in 50ms
  } else if (uart_rx_buffer[0] == myAddress && 
             uart_rx_buffer[1] == 0x05 &&
             uart_rx_buffer[2] == crc8_table[crc ^ 0x05])
  {
    uart_state = 1; // tx state
    send_watering = 1; // in main loop immediately
    send_watering_ok = 50; // send last status in 50ms
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
  }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_state == 1) {
    uart_state = 0;
    ReceiveNextBuffer();
  }  
}

void IDDLE_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(USART1 == huart1.Instance)                                   //Determine whether it is serial port 1
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //Judging whether it is idle interruption
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
            HAL_UART_RxCpltCallback(huart);                          // emulate normal rx, because flow is same
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
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
  if (uart_state == 1) {
    uart_state = 0; // reset state      
    ReceiveNextBuffer();
  }  
}

void SendADCAverages(void) {
  int size = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "ADC|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|%"PRIu32"|STATUS=%"PRIu8",TRES=%"PRIu16",STRES=%"PRIu16",DUR=%"PRIu16"|\n",
          ADC_AVERAGE[0],ADC_AVERAGE[1],ADC_AVERAGE[2],ADC_AVERAGE[3],
          ADC_AVERAGE[4],ADC_AVERAGE[5],ADC_AVERAGE[6],ADC_AVERAGE[7],
          ADC_AVERAGE[8],ADC_AVERAGE[9],ADC_AVERAGE[10],ADC_AVERAGE[11],
          ADC_AVERAGE[12],ADC_AVERAGE[13],ADC_AVERAGE[14],
          watering_status, treshold, safe_treshold, duration);
  if (size > 0) {
    SendBuffer(uart_tx_buffer, size);
  }
}

void SendErrorStatus() {
  int size = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "UART=%"PRIu32"|\n", uart_error);
  if (size > 0) {
    SendBuffer(uart_tx_buffer, size);    
  }
}

void SendOK(uint32_t status) {
  int size = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "OK status=%"PRIu32"\n", status);
  if (size > 0) {
    SendBuffer(uart_tx_buffer, size);    
  }
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_FLASH_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();  
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);   
  HAL_IncTick();
  HAL_GetTick();  
  
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUFFER, 15) != HAL_OK) {
    Error_Handler();        
  }
  
  treshold = ReadVar(2);
  safe_treshold = ReadVar(3);
  duration = ReadVar(4);
  
  while (1)
  {
    if (send_watering > 0) {
      send_watering = 0;
      watering_status = Watering();
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
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
  while(1){}
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
  while(1){}
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

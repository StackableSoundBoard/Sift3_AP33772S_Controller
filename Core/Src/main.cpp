/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 StackableSoundBoard project.
  * All rights reserved, Released under the MIT license 
  ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AP33772S.hpp"
#include "MB85RCxxV.hpp"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <cstdlib>

#include "ntlibc.h"
#include "ntopt.h"
#include "ntshell.h"
#include "stm32c0xx_hal_def.h"
#include "stm32c0xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ntshell_t ntshell;
typedef int (*USRCMDFUNC)(int argc, char **argv);

typedef struct{
  bool isHidden;
  char *cmd;
  char *description;
  USRCMDFUNC func;
} CMD_Table_T;  

typedef struct{
  uint16_t Req_MinVoltage;
  uint16_t Req_MaxVoltage;
  uint16_t Req_Current;
} Request_PDO_T;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UNUSED_VARIABLE(N)  do { (void)(N); } while (0)
#define EEPROM_START_ADDRESS  ((uint32_t)0x0801E000)
#ifndef GIT_COMMIT_HASH
  #define GIT_COMMIT_HASH "Unknown"
#endif

uint8_t UART_Rx_Buffer[64] = {0};
uint8_t UART_Tx_Buffer[64] = {0};
int     RequestedPDOIdx = 0xff;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
MB85RC::MB85RC04V_C mb85rc04(&hi2c1, 0b010);
AP33772S::AP33772S_C ap33772s(&hi2c1);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FLASH_Init(void);
/* USER CODE BEGIN PFP */

static int CMD_help(int argc, char **argv);
static int CMD_info(int argc, char **argv);
static int CMD_CRLF(int argc, char **argv);
static int CMD_EEPROM_Read(int argc, char **argv);
static int CMD_AS33772S_GetSRCPDOList(int argc, char **argv);
// static int CMD_AS33772S_GetDevPDOList(int argc, char **argv);
// static int CMD_AS33772S_SetDevPDO    (int argc, char **argv);
// static int CMD_AS33772S_SetDevPDOList(int argc, char **argv);
// static int CMD_AS33772S_SendRequest  (int argc, char **argv);
// static int CMD_AS33772S_GetStatus    (int argc, char **argv);
// static int CMD_AS33772S_SaveEEPROM   (int argc, char **argv);

static int func_write(const char *buf, int cnt, void *extobj);
static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int func_callback(const char *text, void *extobj);
void       HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

static const CMD_Table_T CommandList[] = {
    { true,  (char*)"help", (char*)"Print help menu", CMD_help},
    { false, (char*)"info", (char*)"Print Device information", CMD_info},
    { false, (char*)"CRLF", (char*)"Set CR/LF Option", CMD_CRLF},    
    { true,  (char*)"EEPR", (char*)"Load EEPROM Data",CMD_EEPROM_Read},
    { false, (char*)"SPDO", (char*)"Show Source PDO data",CMD_AS33772S_GetSRCPDOList},

};

const char GIT_HASH[] = GIT_COMMIT_HASH;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int func_write(const char *buf, int cnt, void *extobj){
  UNUSED(extobj);
  return  (HAL_UART_Transmit(&huart2, (uint8_t*)buf, cnt, HAL_MAX_DELAY)==HAL_OK)? cnt : 0;
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj){
    UNUSED(extobj);
    if (argc == 0) {
        return 0;
    }
    const CMD_Table_T *p = &CommandList[0];
    for (unsigned int i = 0; i < sizeof(CommandList) / sizeof(CommandList[0]); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    // uart_puts("Unknown command found.\r\n");
    return 0;
}


static int func_callback(const char *text, void *extobj){
  ntshell_t *ntshell = (ntshell_t *)extobj;
  UNUSED(ntshell);
  UNUSED(extobj);
  if (ntlibc_strlen(text) > 0){
    ntopt_parse(text, usrcmd_ntopt_callback, 0);
    // snprintf(msg, sizeof(msg), "cmd:%s\r\n", text);
    // HAL_UART_Transmit(&huart2, (const uint8_t*)msg, ntlibc_strlen(msg),HAL_MAX_DELAY);
    // HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
  }

  return 0;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart->Instance == USART2){
    static uint8_t tmp[64];
    if (Size > sizeof(tmp)) Size = sizeof(tmp);
    memcpy(tmp, UART_Rx_Buffer, Size);
    vtrecv_execute(&(ntshell.vtrecv), tmp, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART_Rx_Buffer, sizeof(UART_Rx_Buffer));
  }
}

static int CMD_help(int argc, char **argv){
  UNUSED(argc);
  UNUSED(argv);
  
  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "[Command List]%s", ntshell_newline(&ntshell));
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  for (uint32_t i=0;i<sizeof(CommandList)/sizeof(CommandList[0]);i++){
    if(!CommandList[i].isHidden){
      snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "# %s:\t%s%s", CommandList[i].cmd, CommandList[i].description, ntshell_newline(&ntshell));
      HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
    }
  }
  return 0;
}

static int CMD_info(int argc, char **argv){
  UNUSED(argc);
  UNUSED(argv);
  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "MPU Freq:%ldHz%s", HAL_RCC_GetHCLKFreq(), ntshell_newline(&ntshell));  
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "HAL Ver :%0x%s", (unsigned int)HAL_GetHalVersion(), ntshell_newline(&ntshell));  
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "Rev Ver :%0x%s", (unsigned int)HAL_GetREVID(), ntshell_newline(&ntshell));  
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  return 0;
}

static int CMD_CRLF(int argc, char **argv){
  if(argc==2){
    if(ntlibc_strncmp(argv[1], "CRLF", 5)==0)       {ntshell_crlf(&ntshell, true, true); return 0;}
    else if  (ntlibc_strncmp(argv[1], "LF", 5)==0)  {ntshell_crlf(&ntshell, false, true); return 0;}
  }
    snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "CRLF [LF|CRLF]%s", ntshell_newline(&ntshell));
    HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
    return -1;
}

static int CMD_EEPROM_Read(int argc, char **argv){
  if(argc ==2){
    uint32_t Address = strtol(argv[1], NULL, 16);;
      if(Address >= EEPROM_START_ADDRESS && Address < EEPROM_START_ADDRESS+0x2000){
      uint32_t val = *(volatile uint32_t*)Address;
      snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "0x%08X 0x%08X %s", (unsigned int)Address, (unsigned int)val, ntshell_newline(&ntshell));
      HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
    } 
    return 0;
  }
  return -1;
}

static int CMD_AS33772S_GetSRCPDOList(int argc, char **argv){
  if(argc == 2){
    if(ntlibc_strncmp(argv[1], "?", 2)==0){
      snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "Num of Source PDOs %d%s", ap33772s.FindPDO_Nums(), ntshell_newline(&ntshell));
      HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
    }else{
      uint32_t Itr = strtol(argv[1], NULL, 10);
      for(uint32_t i=0;i<Itr;i++){
        uint32_t VoltCoef = ((i>=7)&ap33772s.EPRStatus()) ? 2 : 1; // Index:8-14 is EPR(200mV step)
        if(ap33772s.SrcPDO_List[i].Type==AP33772S::ADPO) {
          snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "[%2ld]\tAPDO\t%3d-%3d [100mV], %3d[mA] %s%s",i, 
            (ap33772s.SrcPDO_List[i].VoltageMax==1)? 33:50, 
            ap33772s.SrcPDO_List[i].VoltageMax, 
            ap33772s.SrcPDO_List[i].MaxCurrent(ap33772s.SrcPDO_List[i].CurrentMax), 
            (i==RequestedPDOIdx)?"*":"",
            ntshell_newline(&ntshell)
          );
        HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
        }else {
          snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "[%2ld]\tFixed\t%7d [100mV], %3d[mA]%s%s",
            i, 
            ap33772s.SrcPDO_List[i].VoltageMax * VoltCoef, 
            ap33772s.SrcPDO_List[i].MaxCurrent(ap33772s.SrcPDO_List[i].CurrentMax), 
            (i==RequestedPDOIdx)?"*":"",
            ntshell_newline(&ntshell)
          );
        HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);
        
        }
      }
    }
  }
  return -1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_FLASH_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LD08_GPIO_Port, LD08_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD11_GPIO_Port, LD11_Pin, GPIO_PIN_RESET);

  uint8_t buf = 0;
  uint8_t vreq[2] ={0};

  // USB-Cコネクタを刺して起動した場合に何故か1回だとうまくいかないため2回発行する
  for(int ii=0;ii<2;ii++){
    HAL_Delay(300);
    ap33772s.Read_SrcPDO(true);
    ap33772s.SetVout(false);
    ap33772s.WaitResponse(15);
  }

  Request_PDO_T Request_PDO={65, 75, 2000};
  uint8_t PDOIdx = 0xff;
  uint16_t ADPOVolatge100mV = 0;
  bool IsAPDO = true;
  PDOIdx = ap33772s.FindPDO_ADPO(Request_PDO.Req_MinVoltage, Request_PDO.Req_MaxVoltage, Request_PDO.Req_Current, AP33772S::MinWatt, ADPOVolatge100mV);
  if(PDOIdx == 0xff){
    IsAPDO = false;
    PDOIdx = ap33772s.FindPDO_Fixed(Request_PDO.Req_MinVoltage, Request_PDO.Req_MaxVoltage, Request_PDO.Req_Current, AP33772S::MinWatt);
  } 
  
  if(PDOIdx != 0xff){
    for(int ii=0;ii<2;ii++){
      if(IsAPDO) buf = ap33772s.ReqAPDO(PDOIdx, ADPOVolatge100mV, Request_PDO.Req_Current);
      else       buf = ap33772s.ReqFixed(PDOIdx, Request_PDO.Req_Current);
      
      ap33772s.WaitResponse();
    }
    RequestedPDOIdx = ap33772s.RequestedPDO_Idx();
    ap33772s.SetVout(true);
    ap33772s.WaitResponse(15);
    HAL_GPIO_TogglePin(LD11_GPIO_Port, LD11_Pin);
  }else {
    // Cannot find PDO, status=fail
    HAL_GPIO_TogglePin(LD08_GPIO_Port, LD08_Pin);
  }

  // HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, AP33772S::REG::PD_MSGRLT>>8, 1, &buf, 1, 100);
  // HAL_I2C_Mem_Read(&hi2c1, 0x52<<1, AP33772S::REG::VREQ>>8, 1, vreq, 2, 100);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART_Rx_Buffer, sizeof(UART_Rx_Buffer));
  ntshell_init(
      &ntshell,
      NULL,
      func_write,
      func_callback,
      (void *)&ntshell);
      
  ntshell_crlf(&ntshell, true, true);

  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "%sAP33772S-SinkEval Rev:%s%s", ntshell_newline(&ntshell), GIT_HASH, ntshell_newline(&ntshell));
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "# Copyright 2025 rooty19 (https://github.com/rooty19)%s", ntshell_newline(&ntshell), ntshell_newline(&ntshell));
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  snprintf((char*)UART_Tx_Buffer, sizeof(UART_Tx_Buffer), "# Type \"CRLF [LF|CRLF]\" to change EOL.%s", ntshell_newline(&ntshell));
  HAL_UART_Transmit(&huart2, UART_Tx_Buffer, ntlibc_strlen((const char*)UART_Tx_Buffer), HAL_MAX_DELAY);

  ntshell_set_prompt(&ntshell, "> ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int cc = 0;
  char msg[32] = {0};
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FLASH Initialization Function
  * @param None
  * @retval None
  */
static void MX_FLASH_Init(void)
{

  /* USER CODE BEGIN FLASH_Init 0 */

  /* USER CODE END FLASH_Init 0 */

  /* USER CODE BEGIN FLASH_Init 1 */

  /* USER CODE END FLASH_Init 1 */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FLASH_Lock() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FLASH_Init 2 */

  /* USER CODE END FLASH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD08_Pin|LD11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD08_Pin LD11_Pin */
  GPIO_InitStruct.Pin = LD08_Pin|LD11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

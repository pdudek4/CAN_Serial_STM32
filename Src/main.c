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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

static CAN_TxHeaderTypeDef CANTxh;
static CAN_RxHeaderTypeDef CANRxh;
uint32_t CAN_stdid;
uint32_t txmailbox;

	typedef struct {
		
		uint16_t ID;
		uint8_t data[8];
		uint8_t DLC;
	} canTxData_t;
	

	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
	
void CAN_filterConfig(uint32_t*, bool active);
void SendUARTCanMsg(char*);
void ProcessUCMsg(void);
void ProcessUCFilter(void);
void ChangeFilter(uint32_t* canFilter, bool active, volatile bool* state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	
	uint8_t canRxDataBuf[8];
	uint8_t canTxbuf[30];
	char canUmsg[24];
	volatile bool toSendCAN = false;
	volatile bool toSendUART = false;
	volatile bool newFilter = false;
	volatile bool deactFilter = false;
	canTxData_t canTxData;
	uint32_t CANfilter[4]  = {0x300, 0x301, 0x302, 0x303};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CANTxh.IDE = CAN_ID_STD;
	CANTxh.DLC = 8;
	CANTxh.RTR = CAN_RTR_DATA;
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_filterConfig(CANfilter, CAN_FILTER_ENABLE);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	HAL_UART_Receive_IT(&huart2, canTxbuf, 22);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(toSendCAN){
			CANTxh.StdId = canTxData.ID;
			HAL_CAN_AddTxMessage(&hcan1, &CANTxh, canTxData.data, &txmailbox);
			toSendCAN = false; // do zmiany do przerwania cantxcplt
		}
		if(toSendUART){
			toSendUART = false;
			SendUARTCanMsg(canUmsg);
		}
		if(newFilter){
			ChangeFilter(CANfilter, CAN_FILTER_ENABLE, &newFilter);
		}
		if(deactFilter){
			ChangeFilter(CANfilter, CAN_FILTER_DISABLE, &deactFilter);
		}


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CANRxh, canRxDataBuf);
	toSendUART = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if('M' == canTxbuf[0] && ';' == canTxbuf[21]){
		ProcessUCMsg();
	}
	if('F' == canTxbuf[0] && ';' == canTxbuf[21]){
		ProcessUCFilter();
	}
	HAL_UART_Receive_IT(&huart2, canTxbuf, 22);
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
}

void SendUARTCanMsg(char* msg)
{
  sprintf(msg, "M%3x,%02x%02x%02x%02x%02x%02x%02x%02x;\r\n", CANRxh.StdId, 
					canRxDataBuf[0], canRxDataBuf[1], canRxDataBuf[2], canRxDataBuf[3],
					canRxDataBuf[4], canRxDataBuf[5], canRxDataBuf[6], canRxDataBuf[7]);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, 24);
}

void CAN_filterConfig(uint32_t* canFilter, bool active)
{
	CAN_FilterTypeDef filterConfig;
	//skala 32 bit jest spoko do Ext ID, do Std ID najlepiej 16 bit, wtedy mamy 2x wiecej ID 
	//w skali 16 bit high i low oznacza 2 osobne ID
	//w skali 32 bit high i low oznacza MSB i LSB danego 32 bitowego rejestru
	//w trybie ID LIST oraz skali 16 bit w kazdym banku mozna ustawic 4 filtry Std ID
	//w trybie ID LIST FilterMask oraz FilterId dotyczy roznych ID
	//w trybie ID MASK FilterMask dotyczy maski a FilterId dotyczy ID
	filterConfig.FilterBank = 1;
	filterConfig.FilterActivation = active;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = (canFilter[0] << 5);
	filterConfig.FilterIdLow = (canFilter[1] << 5);
	filterConfig.FilterMaskIdHigh = (canFilter[2] << 5);
	filterConfig.FilterMaskIdLow = (canFilter[3] << 5);
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

void ProcessUCMsg(void)
{
	char tmp[4];
	char *pEnd;
	strncpy(tmp, (char*)&canTxbuf[1], 3);
	canTxData.ID = (int)strtol(tmp, &pEnd, 16);
		
	for(int i=0; i<8; i++){
		strncpy(tmp, (char*)&canTxbuf[2*i+5], 2);
		tmp[2] = '\0';
		canTxData.data[i] = (int)strtol(tmp, &pEnd, 16);
	}
	toSendCAN = true;
}

void ProcessUCFilter(void)
{
	char tmp[4];
	char *pEnd;
	
	if('D' == canTxbuf[1]){
		deactFilter = true;
	}
	else if('C' == canTxbuf[1]){
		for(int i=0; i<4; i++){
			strncpy(tmp, (char*)&canTxbuf[3*i+5], 3);
			tmp[3] = '\0';
			CANfilter[i] = (int)strtol(tmp, &pEnd, 16);
		}
		newFilter = true;
	}
	
}

void ChangeFilter(uint32_t* canFilter, bool active, volatile bool* state)
{
	HAL_CAN_Stop(&hcan1);
	CAN_filterConfig(canFilter, active);
	HAL_CAN_Start(&hcan1);
	*state = false;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

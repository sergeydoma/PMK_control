/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

HAL_StatusTypeDef RTF; // test i2c
HAL_StatusTypeDef WTF; // test i2c	
volatile uint8_t arrI2c[10] = {0, 0, 0};
uint8_t arrI2c_R[4][12];
uint32_t addr;
uint8_t arrI2c_T[4][12];
uint8_t block;
uint8_t lanSelect;
uint32_t lanCurrent = 0;
_Bool blockSetEEPROM = 0;
_Bool I2C_HV_off = 1;
//_Bool HV_on = 0;
uint8_t readyHV[4];
uint32_t delayHV;
uint8_t modeHV =0;
_Bool tempHv;
int hv;
int HV_state = -1;
uint32_t delayHV = 0;
//_Bool test_Bipolar;
//_Bool bipolar;
uint32_t led_hv_dinamic = 0;
uint8_t hvAllarm = 0;
//			_Bool hvLoad =0;
//uint16_t hvLoadCurrent=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t xBuffer[4]; // сканирование плат
uint8_t yBuffer[4][10]; // запись ID устройства
uint8_t zBuffer[4][10];
uint32_t pBuffer;
int hvPool = 0; // при 0 на выход идет -100 В
int poolState = -1;
int hvOut =0; 	// 	при 0 выход закорочен на землю
int outState = -1;
uint16_t indicator_alarm = 0;
uint16_t alarm[4]= {0, 0, 0, 0};

#define I2C1_DEVICE_ADDRESS      0x50   /* A0 = A1 = A2 = 0 */
#define MEMORY_ADDRESS      			0x08 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2C_MasterTxCpltCallback()
{
//	RTF = HAL_I2C_Master_Transmit_IT(&hi2c2, 2, masterAddr, 3);
//////	HAL_GPIO_WritePin(GPIOC, DIG0_Pin, 1);
	
	block = 1;
//	RTF = HAL_I2C_Master_Receive_DMA(&hi2c2, 2,arrRes,5);//, 2000);
//	for (int i=0; i<200; i++){}
//  HAL_I2C_Master_Receive_IT(&hi2c2, (1<< 1),  arrI2c, 2);
}
void HAL_I2C_MasterRxCpltCallback()
{
	HAL_IWDG_Refresh(&hiwdg);
	
//////	HAL_GPIO_WritePin(GPIOC, DIG0_Pin, 1);
	block = 0;
//   HAL_I2C_Master_Transmit_IT(&hi2c2, (1<< 1),  masterAddr, 2);
	for (int i=0; i<100; i++){}
}
void HAL_I2C_ErrorCallback()
{
	block = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i; 
	
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
  MX_I2C2_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim7);
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim14);
HAL_TIM_Base_Start_IT(&htim17);
//HAL_TIM_Base_Start_IT(&htim16);
//RTF = HAL_I2C_Master_Transmit_IT(&hi2c2, 0x02, masterAddr, 4);

HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, 1);

  HAL_GPIO_WritePin(P0_LED1_GPIO_Port, P0_LED1_Pin, 1);
  HAL_GPIO_WritePin(P0_LED2_GPIO_Port, P0_LED2_Pin, 0);
  HAL_GPIO_WritePin(P1_LED1_GPIO_Port, P1_LED1_Pin, 1);
  HAL_GPIO_WritePin(P1_LED2_GPIO_Port, P1_LED2_Pin, 0);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_Y_Pin, 1);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_G_Pin, 0);

  HAL_GPIO_WritePin(GPIOC, DIG1_Pin, 1);
  for(int i=0; i<500; i++)
	  HEX_digit(8, DIG0_Pin);

  HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, 0);
  HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, 1);
  HAL_GPIO_WritePin(P0_LED1_GPIO_Port, P0_LED1_Pin, 0);
  HAL_GPIO_WritePin(P0_LED2_GPIO_Port, P0_LED2_Pin, 1);
  HAL_GPIO_WritePin(P1_LED1_GPIO_Port, P1_LED1_Pin, 0);
  HAL_GPIO_WritePin(P1_LED2_GPIO_Port, P1_LED2_Pin, 1);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_Y_Pin, 0);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_G_Pin, 1);

  HAL_GPIO_WritePin(GPIOC, DIG0_Pin, 1);
  for(int i=0; i<500; i++)
	  HEX_digit(8, DIG1_Pin);

  HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, 0);

  HAL_GPIO_WritePin(P0_LED1_GPIO_Port, P0_LED1_Pin, 1);
  HAL_GPIO_WritePin(P0_LED2_GPIO_Port, P0_LED2_Pin, 1);
  HAL_GPIO_WritePin(P1_LED1_GPIO_Port, P1_LED1_Pin, 1);
  HAL_GPIO_WritePin(P1_LED2_GPIO_Port, P1_LED2_Pin, 1);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_Y_Pin, 1);
  HAL_GPIO_WritePin(MDI_Y_GPIO_Port, MDI_G_Pin, 1);


  HAL_GPIO_WritePin(DIG0_GPIO_Port, DIG0_Pin, 0);
  HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, 0);
  GPIOC->BRR = 0xff;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_I2C_Mem_Read(&hi2c2, (uint16_t) I2C1_DEVICE_ADDRESS<<1, MEMORY_ADDRESS, 1, xBuffer, 4, 5); //read memory address 08
//	RTF = HAL_I2C_Master_Transmit_IT(&hi2c2, 2, masterAddr, 3);
//if ((HAL_GPIO_ReadPin(A2_GPIO_Port, A2_Pin)==0))
//{HAL_GPIO_WritePin(Bipolar_GPIO_Port, Bipolar_Pin, 0);}
//else
//{{HAL_GPIO_WritePin(Bipolar_GPIO_Port, Bipolar_Pin, 0);}
  

while (1)
  {

//		HAL_GPIO_WritePin(Bipolar_GPIO_Port, Bipolar_Pin, bipolar);
	
	addr = ((~GPIOA->IDR & 0xff)&0xFC)+0x01;	//230724
		
	  // display current modbus address
	  if ((alarm[0] != 0) & (alarm[1] == 0) & (alarm[2] == 0) &(alarm[3] == 0))
		{
		indicator_alarm = alarm[0];
		}
		else if ((alarm[0] == 0) & (alarm[1] != 0) & (alarm[2] == 0) &(alarm[3] == 0))
		{
		indicator_alarm = alarm[1];
		}
		else if ((alarm[0] == 0) & (alarm[1] == 0) & (alarm[2] != 0) &(alarm[3] == 0))
		{
		indicator_alarm = alarm[2];
		}
		else if ((alarm[0] == 0) & (alarm[1] == 0) & (alarm[2] == 0) &(alarm[3] != 0))
		{
		indicator_alarm = alarm [3];
		}
		else
		{
		indicator_alarm = 0;
		}
		
		
		//		indicator_alarm = 0x21;   2 ошибка 1 авария / предупреждение
		
		if (indicator_alarm != 0)
				{
					HEX_digit(indicator_alarm >> 4, DIG1_Pin);
					HEX_alarm(indicator_alarm & 15, DIG0_Pin);						
			}
		else
				{
					HEX_digit(addr >> 4, DIG1_Pin);
					HEX_digit(addr & 15, DIG0_Pin);		
				}
		
		
	  // process high voltage button
	  hv = HAL_GPIO_ReadPin(SW_HV_GPIO_Port, SW_HV_Pin)==0;

//		hv = hv & HV_on; //!I2C_HV_off;// 20230913

	  if (hv  != HV_state) 
			{
		  // button of high voltage changed
		  HAL_GPIO_WritePin(EN_HV_GPIO_Port, EN_HV_Pin, hv);//hv);	
					
		  HV_state = hv;
			}
			
		if (hvPool != poolState)
		{
			HAL_GPIO_WritePin(HV_POL_GPIO_Port, HV_POL_Pin, hvPool);
			poolState = hvPool;		
		}
		
		if (hvOut != outState)
		{
			HAL_GPIO_WritePin(HV_OUT_GPIO_Port, HV_OUT_Pin, hvOut);
			outState = hvOut;		
		}
		

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//****function transmit Memory	
	//*************************** Блок записи номера шасси начало ***************************** //
				uint8_t masterN = 0xFF;
				for(int i=0; i<5; i++)
				{
					if (((arrI2c_R[i][8]<<8)|(arrI2c_R[i][9])) == 1) // определяем с какой платы вызов
					{
						masterN = i;
						break;
					}
					else
					{
						masterN = 0xFF;
					}
						
				}
				if ((masterN <5)& !blockSetEEPROM)
				{
					yBuffer[masterN][0] = arrI2c_R[masterN][4];
					yBuffer[masterN][1] = arrI2c_R[masterN][5];
					yBuffer[masterN][2] = arrI2c_R[masterN][6];
					yBuffer[masterN][3] = arrI2c_R[masterN][7];				
					
        HAL_I2C_Mem_Write(&hi2c2, (uint16_t) I2C1_DEVICE_ADDRESS<<1, MEMORY_ADDRESS, 1, yBuffer[masterN], 4, 5);	//write to memory address 08 
				HAL_Delay(5000); //system wait
				// Проверка записи
//				yBuffer[masterN][0]=0;yBuffer[masterN][1]=0;yBuffer[masterN][2]=0;yBuffer[masterN][3]=0;yBuffer[masterN][4]=0;yBuffer[masterN][5]=0;yBuffer[masterN][6]=0;yBuffer[masterN][7]=0;yBuffer[masterN][8]=0;yBuffer[masterN][9]=0;
				HAL_I2C_Mem_Read(&hi2c2, (uint16_t) I2C1_DEVICE_ADDRESS<<1, MEMORY_ADDRESS, 1, zBuffer[masterN], 4, 5); //read memory address 08
				for(int i=0; i<4; i++)
				{
						if (yBuffer[masterN][i] != zBuffer[masterN][i])
						{
							arrI2c_T[masterN][5] = 0xF0;
							break;
						}
						else
						{
							arrI2c_T[masterN][5] = 0x0F;
						}
				}
							arrI2c_T[masterN][6] = yBuffer[masterN][0];
							arrI2c_T[masterN][7] = yBuffer[masterN][1];
							arrI2c_T[masterN][8] = yBuffer[masterN][2];
							arrI2c_T[masterN][9] = yBuffer[masterN][3];
				
//							if (hv == 0)
//							{
//								arrI2c_T[masterN][6] = 1; //yBuffer[masterN][0];							
//							}
//							 else
//							{
//								arrI2c_T[masterN][6] = 0; //yBuffer[masterN][0];							
//							}	 
								
				
					blockSetEEPROM = 1;
				}
				
	//*************************** Блок записи номера шасси конец ***************************** //
//       
//***********************************************

// чередование плат
				arrI2c_T[lanSelect][0]=addr;
				arrI2c_T[lanSelect][1]=xBuffer[0];
				arrI2c_T[lanSelect][2]=xBuffer[1];
				arrI2c_T[lanSelect][3]=xBuffer[2];
				arrI2c_T[lanSelect][4]=xBuffer[3];
				arrI2c_T[lanSelect][10]= modeHV; // hv;// 230915
//				arrI2c_T[lanSelect][11]= hvAllarm;
				
//				WTF = HAL_I2C_Master_Transmit_DMA(&hi2c2, 2, masterAddr,5);//, 2000);
				for (int i=0; i<100; i++){}
				
				
					
//				RTF = HAL_I2C_Master_Receive_DMA(&hi2c2, 2, masterAddr,5);//, 2000);
				
				
				
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00101D2D;
  hi2c2.Init.OwnAddress1 = 6;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 799;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIG0_Pin|DIG1_Pin|SEG0_Pin|SEG1_Pin
                          |SEG2_Pin|SEG3_Pin|SEG4_Pin|SEG5_Pin
                          |SEG6_Pin|SEG7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Bipolar_Pin|HV_POL_Pin|HV_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_HV_Pin|P0_LED1_Pin|P0_LED2_Pin|P1_LED1_Pin
                          |P1_LED2_Pin|MDI_G_Pin|MDI_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_HV_GRN_Pin|LED_HV_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIG0_Pin DIG1_Pin LED_HV_GRN_Pin LED_HV_RED_Pin */
  GPIO_InitStruct.Pin = DIG0_Pin|DIG1_Pin|LED_HV_GRN_Pin|LED_HV_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin SEG3_Pin
                           SEG4_Pin SEG5_Pin SEG6_Pin SEG7_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|SEG3_Pin
                          |SEG4_Pin|SEG5_Pin|SEG6_Pin|SEG7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Bipolar_Pin HV_POL_Pin HV_OUT_Pin */
  GPIO_InitStruct.Pin = Bipolar_Pin|HV_POL_Pin|HV_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin A2_Pin A3_Pin A4_Pin
                           A5_Pin A6_Pin A7_Pin */
  GPIO_InitStruct.Pin = A1_Pin|A2_Pin|A3_Pin|A4_Pin
                          |A5_Pin|A6_Pin|A7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_HV_Pin P0_LED1_Pin P0_LED2_Pin P1_LED1_Pin
                           P1_LED2_Pin MDI_G_Pin MDI_Y_Pin */
  GPIO_InitStruct.Pin = EN_HV_Pin|P0_LED1_Pin|P0_LED2_Pin|P1_LED1_Pin
                          |P1_LED2_Pin|MDI_G_Pin|MDI_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_HV_Pin */
  GPIO_InitStruct.Pin = SW_HV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_HV_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM17)
	{
		if(block==0)
		{
		WTF = HAL_I2C_Master_Transmit_DMA(&hi2c2, (lanSelect+1), arrI2c_T[lanSelect],12);//, 2000);
		}
		else
		{
		// Перезапуск сторожевого таймера
//		HAL_WWDG_Refresh(&hwwdg);	
			RTF = HAL_I2C_Master_Receive_DMA(&hi2c2, (lanSelect+1),arrI2c_R[lanSelect],12);//, 2000); 
			alarm[0] = arrI2c_R[0][11];	
			alarm[1] = arrI2c_R[1][11];
			alarm[2] = arrI2c_R[2][11];
			alarm[3] = arrI2c_R[3][11];
		}
		
		
	}
	
	if(htim->Instance == TIM7)
	{
		
		
		
	}
	if(htim->Instance == TIM6)
	{
		if(lanCurrent > _timeLanC)
		{lanCurrent = 0;}
		else 
		{lanCurrent++;}
		switch (lanCurrent)
    {
    	case 00:
				lanSelect = 0;
    		break;
    	case 10:
				lanSelect = 1;
    		break;
//			case 200:
//				lanSelect = 3;
//    		break;
//    	case 300:
//				lanSelect = 4;
//    		break;
    }
		
		if ((arrI2c_R[0][0]) | arrI2c_R[1][0]|arrI2c_R[2][0]|arrI2c_R[3][0]) // 230915 
			{
				hvAllarm = 1;
////			HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, GPIO_PIN_SET);
//////				I2C_HV_off = 1;			
			}
			else
			{
				hvAllarm =0;
////			HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, GPIO_PIN_RESET);
//////				I2C_HV_off = 0;
			}	
			
			
			if (arrI2c_R[0][1]|arrI2c_R[1][1]|arrI2c_R[2][1]|arrI2c_R[3][1])
			{HAL_GPIO_WritePin(P0_LED1_GPIO_Port, P0_LED1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(P1_LED1_GPIO_Port, P1_LED1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(P0_LED2_GPIO_Port, P0_LED2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(P1_LED2_GPIO_Port, P1_LED2_Pin,GPIO_PIN_SET);}
			else
			{HAL_GPIO_WritePin(P0_LED1_GPIO_Port, P0_LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(P1_LED1_GPIO_Port, P1_LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(P0_LED2_GPIO_Port, P0_LED2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(P1_LED2_GPIO_Port, P1_LED2_Pin,GPIO_PIN_RESET);}
			

	}
	if(htim->Instance == TIM14)
	{
//		tempHv = HAL_GPIO_ReadPin(A1_GPIO_Port, A1_Pin)==0; // 1 если аварии нет
//		if (tempHv)
//		{hvLoad = 0;}//Нужно наоборот
//		else if ((modeHV==4) | (modeHV==8))
//		{hvLoad = 1;}
		
		switch (modeHV)
    {
    	case 0:
				hvOut =0;
				hvPool = 0;
				if (timeDel(_timePause))
				{
					modeHV = 1;					
				}
    		break;
    	case 1:
				hvOut =0;
				hvPool = 0;
				if (timeDel(_timeMeasureVolt))
				{
					modeHV = 2;					
				}
    		break;
			case 2:
				hvOut =0;
				hvPool = 0;
				if (timeDel(_timeDeley))
				{
					modeHV = 3;					
				}
				break;
			case 3:
				hvOut =1;
				hvPool = 0;

				if (timeDel(_timePause))
				{
					modeHV =4;					
				}
				break;
			case 4:
				hvOut =1;
				hvPool = 0;			
				if(timeDel(_timeMeasure))
				{
					modeHV = 5;
				}
				break;
				
			case 5:
				hvOut =1;
				hvPool = 0;

				if(timeDel(_timeDeley))
				{
					modeHV = 6;
				}
				break;
			case 6:						// пауза для снятия - перед подачей +
				hvOut =0;
				hvPool = 1;
				if(timeDel(_timeDeley))
				{
					modeHV = 7;
				}
				break;
    	case 7:
				hvOut =1;
				hvPool = 1;

				if (timeDel(_timePause))
				{
					modeHV = 8;
				}
    		break;
    	case 8:
				hvOut =1;
				hvPool = 1;
			
				if (timeDel(_timeMeasure))
				{
					modeHV = 9;
				}
    		break;
			case 9:
				hvOut =1;
				hvPool = 1;
				if (timeDel(_timeDeley))
				{
					modeHV = 10;
				}
				break;
			case 10:											// пауза для снятия + перед подачей -
				hvOut = 0;
				hvPool = 1;
				if (timeDel(_timeDeley))
				{
					modeHV = 0;
				}
				break;
			
			
								
    	default:
				modeHV = 0;
    		break;
    }
			

	}
	if(htim->Instance == TIM16)
	{	
		
		
		

			
		if (hvAllarm)
			{
				HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin,1);
				HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, 0);//);
			}
		else if (hv & hvOut)
			{
				HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, 1);//);
				HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, 0);
			}	
		else
			{
			HAL_GPIO_WritePin(LED_HV_GRN_GPIO_Port, LED_HV_GRN_Pin, 1);//);
			HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, 1);
			}	
		if((led_hv_dinamic>0)&(led_hv_dinamic<11)& (hvAllarm == 0))
			{HAL_GPIO_WritePin(LED_HV_RED_GPIO_Port, LED_HV_RED_Pin, 0);}
		else if(led_hv_dinamic>=14)
			{led_hv_dinamic =0;}
			led_hv_dinamic++;
		
	}
		
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

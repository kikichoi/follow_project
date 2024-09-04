/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include <stdio.h> // sprintf 등
#include <math.h>
#include <string.h> // strlen(const char *s) 등
#include <stdlib.h> //  exit() atoi() rand() srand() system()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RxBuf_SIZE_uart1 8
#define MainBuf_SIZE_uart1 8
#define RxBuf_SIZE_uart2 8
#define MainBuf_SIZE_uart2 8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t RxBuf_uart1[RxBuf_SIZE_uart1];
uint8_t MainBuf_uart1[MainBuf_SIZE_uart1];
extern DMA_HandleTypeDef hdma_usart6_rx;
uint8_t RxBuf_uart2[RxBuf_SIZE_uart2];
uint8_t MainBuf_uart2[MainBuf_SIZE_uart2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

//printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
 * @brief Retargets the C library printf function to the USART
 * @param None
 * @retval None
 */

//printf
PUTCHAR_PROTOTYPE{
   if (ch == '\n') HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
   HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
   return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float rx_value_uart1 = 0.0;
float rx_value_uart2 = 0.0;

//average filter
#define MASK_LENGTH 5
uint16_t raw_array_1[MASK_LENGTH] = {0,};
uint16_t raw_array_index_1 = 0;
uint16_t raw_array_2[MASK_LENGTH] = {0,};
uint16_t raw_array_index_2 = 0;

//average value
float ch1_value0 = 0.0;
float ch1_value1 = 0.0;
float ch1_value2 = 0.0;
float ch1_value3 = 0.0;
float ch1_value4 = 0.0;

float ch2_value0 = 0.0;
float ch2_value1 = 0.0;
float ch2_value2 = 0.0;
float ch2_value3 = 0.0;
float ch2_value4 = 0.0;
float ch2_value5 = 0.0;

float sum_1 = 0.0;
float sum_2 = 0.0;

uint8_t TxBuf_1[8] = {0,};
uint8_t TxBuf_2[8] = {0,};

//UART_RX DMA
uint16_t oldPos_uart1 = 0;
uint16_t newPos_uart1 = 0;
uint16_t oldPos_uart2 = 0;
uint16_t newPos_uart2 = 0;


//////////////////////////MOTOR///////////////////////
#define straight 0xaa
#define right 0xbb
#define left 0xcc

//모터드라이버 엔코더 값 기준으로 직진 보정한 pwm 값
#define motor_pwm1_L 320
#define motor_pwm2_R 340

int speed_ch1,speed_ch2;
uint8_t MOTOR_SPEED_UWB[12] = {0,};

unsigned short MOTOR_CRC(unsigned char *addr, int num)
{
   unsigned short MOTOR_CRC = 0xFFFF;
   int i;
   while (num--)
   {
      MOTOR_CRC ^= *addr++;
      for (i = 0; i < 8; i++)
      {
         if (MOTOR_CRC & 1)
         {
            MOTOR_CRC >>= 1;
            MOTOR_CRC ^= 0xA001;
         }
         else
         {
            MOTOR_CRC >>= 1;
         }
      }
   }
   return MOTOR_CRC;
}


void Packet_UWB(int Speed_Value1 , int Speed_Value2, int direction)
{
   MOTOR_SPEED_UWB[0] = 0x11;
   MOTOR_SPEED_UWB[1] = 0x22;

   //uwb_data_1
   MOTOR_SPEED_UWB[2] = (Speed_Value1 >> 8) & 0xFF;
   MOTOR_SPEED_UWB[3] = Speed_Value1 & 0xFF;

   //uwb_data_2
   MOTOR_SPEED_UWB[4] = (Speed_Value2 >> 8) & 0xFF;
   MOTOR_SPEED_UWB[5] = Speed_Value2 & 0xFF;

   unsigned int crcValue = MOTOR_CRC(MOTOR_SPEED_UWB, 6);
   MOTOR_SPEED_UWB[7] = (crcValue >> 8) & 0xFF;
   MOTOR_SPEED_UWB[6] = crcValue & 0xFF;

   MOTOR_SPEED_UWB[8] = 0x08;
   MOTOR_SPEED_UWB[9] = 0x09;

   if(direction == 1)
   {
      MOTOR_SPEED_UWB[10] = straight;
   }
   else if(direction == 2)
   {
      MOTOR_SPEED_UWB[10] = right;
   }
   else if(direction == 3)
   {
      MOTOR_SPEED_UWB[10] = left;
   }

   MOTOR_SPEED_UWB[11] = 0xEE; //address check
   HAL_Delay(1);

   HAL_UART_Transmit(&huart1, MOTOR_SPEED_UWB, 12, 100);
   //HAL_UART_Transmit(&huart2, MOTOR_SPEED_UWB, 12, 100); //terminal
   memset(MOTOR_SPEED_UWB, 0, sizeof(MOTOR_SPEED_UWB));
}

void MOTOR_control()
{
   if(sum_1 <= 0.5 && sum_2 <= 0.5)
   {
      speed_ch1 = 0;
      speed_ch2 = 0;
      Packet_UWB(speed_ch1,speed_ch2 , 1);
      HAL_Delay(1);
      if(sum_1 < 0.3 && sum_2 < 0.3)
      {
    	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Buzzer ON
      }
      else
      {
    	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Buzzer OFF
      }
   }
   else if(sum_1 > 0.5 && sum_2 > 0.5 )
   {
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Buzzer OFF
      if(sum_1 - sum_2 > 0.1)
      {
         speed_ch1 = motor_pwm1_L + 200 ;
         speed_ch2 = motor_pwm2_R   ;
      }
      else if(sum_1 - sum_2 < -0.1)
      {
         speed_ch1 = motor_pwm1_L ;
         speed_ch2 = motor_pwm2_R + 200 ;
      }
      else if(sum_1 >= 1.5 && sum_2 >= 1.5)
      {
         if(sum_1 - sum_2 < 0.2)
         {
            speed_ch1 = motor_pwm1_L + (sum_1*150) + 200 ;
            speed_ch2 = motor_pwm2_R + (sum_2*150);
         }
         else if(sum_1 - sum_2 < -0.2)
         {
            speed_ch1 = motor_pwm1_L + (sum_1*150);
            speed_ch2 = motor_pwm2_R + (sum_2*150) + 200 ;
         }
         else
         {
            speed_ch1 = motor_pwm1_L + (sum_1*150);
            speed_ch2 = motor_pwm2_R + (sum_2*150);
         }
      }
      else
      {
         speed_ch1 = motor_pwm1_L ;
         speed_ch2 = motor_pwm2_R ;
      }
      Packet_UWB(speed_ch1 , speed_ch2 , 1);
      //HAL_Delay(1);
   }
   else if(sum_1 > 0.5 && sum_2 < 0.5 )
   {
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Buzzer OFF
      speed_ch1 = motor_pwm1_L + 50;
      speed_ch2 = motor_pwm2_R + 50;
      Packet_UWB(speed_ch1, speed_ch2 , 2);
   }
   else if(sum_1 < 0.5 && sum_2 > 0.5 )
   {
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Buzzer OFF
      speed_ch1 = motor_pwm1_L + 50;
      speed_ch2 = motor_pwm2_R + 50;
      Packet_UWB(speed_ch1, speed_ch2 , 3);
   }
}
///////////////////////////////MOTOR//////////////////////////////////


unsigned Array_flag_ch1 = 0;
unsigned Array_flag_ch2 = 0;
void insertIntoRawArray_ch1(float value)
{
   if(value >= 200)
   {//데이터 튀는 값 저장X
   }
   else if(value < 200)
   {
      if(raw_array_index_1 == 0) ch1_value0 = value;
      else if(raw_array_index_1 == 1) ch1_value1 = value;
      //else if(raw_array_index_1 == 2) ch1_value2 = value;
      //else if(raw_array_index_1 == 3) ch1_value3 = value;
      //else if(raw_array_index_1 == 4) ch1_value4 = value;
      raw_array_index_1++;

      if (raw_array_index_1 >= 2)
      {
          //sum_1 = ( ch1_value0 + ch1_value1 + ch1_value2 + ch1_value3 + ch1_value4 ) / MASK_LENGTH ;
          sum_1 =  (ch1_value0 + ch1_value1) / 2 ;
          //printf("%f\n\r", sum_1);

          uint8_t TX_sum[50];
          sprintf((char*)TX_sum, "sum_1 : %f \r\n", sum_1);
          HAL_UART_Transmit(&huart2, TX_sum, strlen((const char *)TX_sum), 100); //terminal

          raw_array_index_1 = 0;
          Array_flag_ch1 = 1;
       }
   }
}

void insertIntoRawArray_ch2(float value)
{
   if(value >= 200)
   {//데이터 튀는 값 저장X
   }
   else if(value < 200)
   {
      if(raw_array_index_2 == 0) ch2_value0 = value;
      //else if(raw_array_index_2 == 1) ch2_value1 = value;
      //else if(raw_array_index_2 == 2) ch2_value2 = value;
      //else if(raw_array_index_2 == 3) ch2_value3 = value;

      raw_array_index_2++;

      if (raw_array_index_2 >= 1)
      {
          //sum_2 = ( ch2_value0 + ch2_value1 + ch2_value2 + ch2_value3 ) / 4 ;

          sum_2 = ch2_value0 ;
          //printf("%f\n\r",sum_2);

          uint8_t TX_sum[50];
          sprintf((char*)TX_sum, "sum_2 : %f \r\n", sum_2);
          HAL_UART_Transmit(&huart2, TX_sum, strlen((const char *)TX_sum), 100); //terminal
          raw_array_index_2 = 0;
          Array_flag_ch2 = 1;
      }
   }
}



//UART DMA RX , [UART1 , UART6]
//ESP32_UWB , data RX
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   if (huart->Instance == USART1)
   {
      oldPos_uart1 = newPos_uart1;

      if (oldPos_uart1+Size > MainBuf_SIZE_uart1)
      {
         uint16_t datatocopy = MainBuf_SIZE_uart1-oldPos_uart1;
         memcpy ((uint8_t *)MainBuf_uart1+oldPos_uart1, RxBuf_uart1, datatocopy);

         oldPos_uart1 = 0;
         memcpy ((uint8_t *)MainBuf_uart1, (uint8_t *)RxBuf_uart1+datatocopy, (Size-datatocopy));
         newPos_uart1 = (Size-datatocopy);
      }
      else
      {
         memcpy ((uint8_t *)MainBuf_uart1+oldPos_uart1, RxBuf_uart1, Size);
         newPos_uart1 = Size+oldPos_uart1;
      }

      /* start the DMA again */
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf_uart1, RxBuf_SIZE_uart1);
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
      //HAL_UART_Transmit_IT(&huart2 ,(uint8_t *) RxBuf_uart1,RxBuf_SIZE_uart1);
      sscanf((char *)RxBuf_uart1,"%f",&rx_value_uart1);

      if(rx_value_uart1 >= 0)
      {
         //average filter
            insertIntoRawArray_ch1(rx_value_uart1);
      }
      else insertIntoRawArray_ch1(0); //음수 데이터로 튀는 값은 0 처리

      //ESP32_UWB DATA TEST
      //*************************************************//
      /*if(rx_value_uart1 < 10.0)
      {
         //HAL_GPIO_WritePin(GPIOC, led0_Pin, GPIO_PIN_SET);
      }

      if(rx_value_uart1 >= 10.0)
      {
         //HAL_GPIO_WritePin(GPIOC, led0_Pin, GPIO_PIN_RESET);
      }*/
      //*************************************************//
   }

   if (huart->Instance == USART6)
   {
      oldPos_uart2 = newPos_uart2;  // Update the last position before copying new data

      if (oldPos_uart2+Size > MainBuf_SIZE_uart2)  // If the current position + new data size is greater than the main buffer
      {
         uint16_t datatocopy_2 = MainBuf_SIZE_uart2-oldPos_uart2;  // find out how much space is left in the main buffer
         memcpy ((uint8_t *)MainBuf_uart2+oldPos_uart2, RxBuf_uart2, datatocopy_2);  // copy data in that remaining space

         oldPos_uart2 = 0;  // point to the start of the buffer
         memcpy ((uint8_t *)MainBuf_uart2, (uint8_t *)RxBuf_uart2+datatocopy_2, (Size-datatocopy_2));  // copy the remaining data
         newPos_uart2 = (Size-datatocopy_2);  // update the position
      }

      else
      {
         memcpy ((uint8_t *)MainBuf_uart2+oldPos_uart2, RxBuf_uart2, Size);
         newPos_uart2 = Size+oldPos_uart2;
      }

      /* start the DMA again */
      HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *) RxBuf_uart2, RxBuf_SIZE_uart2);
      __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
      //HAL_UART_Transmit_IT(&huart2 ,(uint8_t *) RxBuf_uart2,RxBuf_SIZE_uart2);
      sscanf((char *) RxBuf_uart2,"%f",&rx_value_uart2);

      if(rx_value_uart2 >= 0)
      {
         //average filter
            insertIntoRawArray_ch2(rx_value_uart2);
      }
      else insertIntoRawArray_ch2(0); //음수 데이터로 튀는 값은 0 처리

      /*if(rx_value_uart2 < 10.0)
      {
        //HAL_GPIO_WritePin(GPIOC, led1_Pin, GPIO_PIN_SET);
      }

      if(rx_value_uart2 >= 10.0)
      {
         //HAL_GPIO_WritePin(GPIOC, led1_Pin, GPIO_PIN_RESET);
      }*/
   }
}


//Buzzer
void Warning_Sound()
{
	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Buzzer ON
	 //HAL_Delay(3000); // 3s ON
	 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);  // Buzzer OFF
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

 /////////////////////stm32 comportmaster check
  HAL_UART_Transmit_IT(&huart2 ,(uint8_t *) RxBuf_uart2,RxBuf_SIZE_uart2);
  //////////////////////////

  //uart_DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf_uart1, RxBuf_SIZE_uart1);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //Half Transfer ,DISABLE
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuf_uart2, RxBuf_SIZE_uart2);
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

  //HAL_UART_Receive_DMA(&huart6, &rx_dma_data6, 8);

  //uart test
  //HAL_UART_Transmit(&huart2, "hello\n\r", 6,100);

  //HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     if( (Array_flag_ch1 == 1) && (Array_flag_ch2 == 1) )
     {
    	MOTOR_control();
        Array_flag_ch1 = 0;
        Array_flag_ch2 = 0;
     }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led0_Pin|led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led0_Pin led1_Pin */
  GPIO_InitStruct.Pin = led0_Pin|led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI0_Pin EXTI1_Pin */
  GPIO_InitStruct.Pin = EXTI0_Pin|EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

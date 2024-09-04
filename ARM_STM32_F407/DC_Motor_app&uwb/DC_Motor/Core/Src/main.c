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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART4_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

unsigned short calc_crc16(unsigned char *data, int len);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE{
   if (ch == '\n') HAL_UART_Transmit(&huart3, (uint8_t*)"\r", 1, 0xFFFF);
   HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);
   return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PI 3.141592
#define WHEEL_DIAMETER 0.34      //34cm
#define WHEEL_RADIUS 0.17
#define MOTOR_REDUCER   21.0   //gamsok bi = 21:1

#define PPR_MOTOR1 14400.0   // 3600*4
#define PPR_MOTOR2 14400.0

unsigned char direction_motor1 = 0;
unsigned char direction_motor2 = 0;
unsigned char direction_m1_check = 0;
unsigned char direction_m2_check = 0;

double wheel_1;
double wheel_2;
double wheel_1_distance;
double wheel_2_distance;
double delta_s;

//pwm variable
int motor1_enc_counter = 0;
int motor2_enc_counter = 0;

uint8_t msg_uwb[100];
uint8_t msg_app[100];

//timer variable
char flag = 0;
char flag_1 = 0;

int pwm_m1  = 0;
int pwm_m2  = 0;
unsigned int pwm_m1_final = 0;
unsigned int pwm_m2_final = 0;

int start_flag = 0;
int start_flag_1 = 0;
int start_flag_2 = 0;

//time period 20ms
#define TIME 0.02

float ENCODER1;
float ENCODER2;

int prev_motor1_enc_counter = 0;
int prev_motor2_enc_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == TIM7)
   {//ENCODER
        start_flag = 1;
        motor1_enc_counter = TIM3->CNT;
        TIM3->CNT = 0;

        start_flag_1 ++;

        if(start_flag_1 >= 5)
        {
           start_flag_2 = 1;
           start_flag_1 = 5;
        }

        // 모터1 방향 ?   ???
     if (motor1_enc_counter <= 30000)
     {
       direction_m1_check = 1;  // ?   ???
     }
     else if (motor1_enc_counter >= 35535)
     {
       motor1_enc_counter = -(65535 - motor1_enc_counter);  // ?   ??? 보정
       direction_m1_check = 0;  // ?   ???
     }
     else
     {
       // ?  ?   값과 비교?  ?   방향 ?   ???
       if (motor1_enc_counter > prev_motor1_enc_counter)
       {
         direction_m1_check = 1;  // ?   ???
       }
       else
       {
         motor1_enc_counter = -(65535 - motor1_enc_counter);  // ?   ??? 보정
         direction_m1_check = 0;  // ?   ???
       }
     }

      motor2_enc_counter = TIM1->CNT;
      TIM1->CNT = 0;

      // 모터2 방향 ?   ???
     if (motor2_enc_counter <= 30000)
     {
       direction_m2_check = 1;  // ?   ???
     }
     else if (motor2_enc_counter >= 35535)
     {
       motor2_enc_counter = -(65535 - motor2_enc_counter);  // ?   ??? 보정
       direction_m2_check = 0;  // ?   ???
     }
     else
     {
       // ?  ?   값과 비교?  ?   방향 ?   ???
       if (motor2_enc_counter > prev_motor2_enc_counter)
       {
         direction_m2_check = 1;  // ?   ???
       }
       else
       {
         motor2_enc_counter = -(65535 - motor2_enc_counter);  // ?   ??? 보정
         direction_m2_check = 0;  // ?   ???
       }
     }

     ENCODER1 = motor1_enc_counter;
     ENCODER2 = motor2_enc_counter;

     //uint8_t TX_ENCODER[100];
     //sprintf((char*)TX_ENCODER, "ENCO_1 : %f, ENCO_2 : %f\r\n",ENCODER1 , ENCODER2);
     //HAL_UART_Transmit(&huart3,TX_ENCODER, strlen((const char *)TX_ENCODER), 100);

      // ?  ?   ?  코더  ??? ?  ?  ?  ?
      //prev_motor1_enc_counter = motor1_enc_counter;
      //prev_motor2_enc_counter = motor2_enc_counter;

      if(motor1_enc_counter - prev_motor1_enc_counter > 170 )
      {
         motor1_enc_counter = prev_motor1_enc_counter ;
      }
      else
      {
         prev_motor1_enc_counter = motor1_enc_counter ;
      }

      if(motor2_enc_counter - prev_motor2_enc_counter > 170 )
      {
        motor2_enc_counter = prev_motor2_enc_counter ;
      }
      else
      {
        prev_motor2_enc_counter = motor2_enc_counter ;
      }

      wheel_1 =  motor1_enc_counter*360.0/302400.0;
      wheel_2 =  motor2_enc_counter*360.0/302400.0;

      wheel_1_distance = WHEEL_RADIUS*wheel_1*2*PI/360.0;
      wheel_2_distance = WHEEL_RADIUS*wheel_2*2*PI/360.0;

      delta_s = (wheel_1_distance+wheel_2_distance)/2*100000;

      motor1_enc_counter = 0;
      motor2_enc_counter = 0;

      start_flag = 0;
   }

}

//CRC-16 function
unsigned short calc_crc16(unsigned char *data, int len) {
    unsigned short crc = 0xFFFF;
    int i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];

        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

//UWB Packet , uart5
//*********************************************************************************************//
// [0x11] [0x22] [speed_L] [speed_L] [speed_R] [speed_R] [CRC] [CRC] [0x08] [0x09] [0x10] [0xEE]
//*********************************************************************************************//

//uart5 ver
uint8_t rx_dma_data5[12] = {0,};
uint8_t UWB_Rx_index = 0;
uint8_t UWB_Rx_buf[12] = {0,};
uint8_t receive_flag_UWB = 0;
uint16_t UWB_PWM_R = 0;
uint16_t UWB_PWM_L = 0;


//uart4 var
uint8_t Rx_index4 = 0;
uint8_t Rx_buf4[100] = {0,};
uint8_t receive_flag4 = 0;
uint8_t app_flag = 0;
uint8_t app_state = 0;
uint16_t app_PWM_R = 0;
uint16_t app_PWM_L = 0;

unsigned char rx_dma_data4 = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
   //UWB packet
   if(huart->Instance == UART5)
   {
      //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
      //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
      //HAL_UART_Transmit(&huart3, "hello\n\r", 6,100);
      for(int i = 0; i<12; i++)
      {
        UWB_Rx_buf[i] = rx_dma_data5[i];
      }
        //UWB_Rx_buf[UWB_Rx_index] = rx_dma_data5[UWB_Rx_index];
        //UWB_Rx_index++;

        if(UWB_Rx_buf[11] == 0xEE) //address check
        {
        	if(app_flag == 1)
        	{
        		receive_flag_UWB = 0;
        	}
        	else
        	{
        		receive_flag_UWB = 1;
        	}
        }

        if(receive_flag_UWB == 1)
        {
         if((UWB_Rx_buf[0] == 0x11) && (UWB_Rx_buf[1] == 0x22))
         {
            if(UWB_Rx_buf[10] == 0xaa)
            {
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
            }
            else if(UWB_Rx_buf[10] == 0xbb)
            {
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,0); //dir_motor2 setting
            }
            else if(UWB_Rx_buf[10] == 0xcc)
            {
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0); //dir_motor1 setting
               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
            }

            UWB_PWM_L = (UWB_Rx_buf[2] << 8) | (UWB_Rx_buf[3]) ;
            UWB_PWM_R = (UWB_Rx_buf[4] << 8) | (UWB_Rx_buf[5]) ;

            if(UWB_PWM_R >= 800)
            {
              UWB_PWM_R = 790;
            }
            else if(UWB_PWM_L >= 800)
            {
              UWB_PWM_L = 790;
            }

            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,UWB_PWM_L); // MOTOR1 , UWB_PWM_L , sum_1
            __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2,UWB_PWM_R); // MOTOR2 , UWB_PWM_R , sum_2
            sprintf((char*)msg_uwb, "%d\t %d\t \r\n", UWB_PWM_L, UWB_PWM_R);
            HAL_UART_Transmit_IT(&huart3, msg_uwb, sizeof(msg_uwb));
            receive_flag_UWB = 0;
         }
      }
        HAL_UART_Receive_DMA(&huart5, rx_dma_data5, 12);
   }

   if(huart->Instance == UART4)
   {
	  //HAL_UART_Transmit(&huart3, "hello\n\r", 6,100);
	  Rx_buf4[Rx_index4++] = rx_dma_data4;

	  if(Rx_buf4[Rx_index4 - 1] == 0x77)
	  {//address check
		receive_flag4 = 1;
		Rx_index4 = 0;
	  }

	  if(receive_flag4 == 1)
	  {
		  if(Rx_buf4[0] == 0xaa)
		  {//ON , UWB_OFF
			  app_flag = 1;
			  app_state++;
			  if(app_state == 1)
			  {//처음으로 버튼 누른 상황
				  app_PWM_R = 0;
				  app_PWM_L = 0;
			  }

			  if(Rx_buf4[1] == 0xfb)
			  {//wait
				  //값 유지
			  }
			  else if(Rx_buf4[1] == 0xfc)
			  {//reset
				 app_PWM_R = 0;
				 app_PWM_L = 0;
			  }
			  else if(Rx_buf4[1] == 0xab)
			  {//speed up
				  if(app_PWM_R >= 450 || app_PWM_L >= 450)
				  {//최고속도 제한
				  	  app_PWM_R = 450;
				  	  app_PWM_L = 450;
				  }
				  else
				  {
				  	  app_PWM_R += 50;
				  	  app_PWM_L += 50;
				  }
			  }
			  else if(Rx_buf4[1] == 0xac)
			  {//speed down
				  if(app_PWM_R <= 50 || app_PWM_L <= 50)
				  {//언더플로우 방지
					  app_PWM_R = 0;
					  app_PWM_L = 0;
				  }
				  else
				  {
					  app_PWM_R -= 50;
					  app_PWM_L -= 50;
				  }
			  }

			  if(Rx_buf4[2] == 0xcc)
			  {//forward
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
			  }
			  else if(Rx_buf4[2] == 0xdd)
			  {//turn left
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0); //dir_motor1 setting
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
			  }
			  else if(Rx_buf4[2] == 0xee)
			  {//turn right
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,0); //dir_motor2 setting
			  }
			  else if(Rx_buf4[2] == 0xfa)
			  {//backward
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0); //dir_motor1 setting
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,0); //dir_motor2 setting
			  }
		  }
		  else if(Rx_buf4[0] == 0xbb)
		  {//OFF , UWB_ON
			  app_flag = 0;
			  app_state = 0;
			  app_PWM_R = 0;
			  app_PWM_L = 0;
		  }

		 //uint8_t TX_TTTT[100];
		 //sprintf((char*)TX_TTTT, "%f, %f,%.4f\r\n",ENCODER2 , ENCODER1 ,yaw_angle);
		 //sprintf((char*)TX_TTTT, "%f, %f\r\n",ENCODER2 , ENCODER1);
		 //HAL_UART_Transmit(&huart3, TX_TTTT, strlen((const char *)TX_TTTT), 100);

		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,app_PWM_L);
		  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2,app_PWM_R);

		  sprintf((char*)msg_app, "%d\t %d\t %d\t \r\n", app_PWM_R, app_PWM_L , app_flag);
		  HAL_UART_Transmit_IT(&huart3, msg_app, sizeof(msg_app));
		  receive_flag4 = 0;
	  }

	   HAL_UART_Receive_DMA(&huart4,&rx_dma_data4, 1);
   }
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_UART5_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); //mc_en set

  //MOTOR1 Setting
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET); // nsleep set
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET); // brk control set
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,200);

  //MOTOR2 Setting
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); // nsleep set
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET); // dir set
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET); // brk control set
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  //__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2,200);

  //UART Setting
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart5,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_TC);

  //-----------------
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2); // Start encoder mode
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2); // Start encoder mode

  //HAL_Delay(2000);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,0); // test motor1
  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2,0); // test motor2
  //MOTOR1, MOTOR2 20ms timer start
  HAL_TIM_Base_Start_IT(&htim7); // Timer start
  //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);

  HAL_UART_Receive_DMA(&huart5, rx_dma_data5, 12); //uwb
  HAL_UART_Receive_DMA(&huart4,&rx_dma_data4, 1); //app

  //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,320); // test motor1
  //__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2,340); // test motor2
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //dir_motor1 setting
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,1); //dir_motor2 setting
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_UART_Transmit(&huart3, "hello\n\r", 6,100);
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* UART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
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
  htim7.Init.Prescaler = 840-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000-1;
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
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 3499;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|BRK_CONTROL_2_Pin|BRK_CONTORL_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port, nSLEEP_M2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_DC_M2_Pin|DIR_DC_M1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port, nSLEEP_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MC_EN_GPIO_Port, MC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CHK_MC_EN_Pin CHK_MC_DIS_Pin */
  GPIO_InitStruct.Pin = CHK_MC_EN_Pin|CHK_MC_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin BRK_CONTROL_2_Pin BRK_CONTORL_1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|BRK_CONTROL_2_Pin|BRK_CONTORL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH1_Pin SWITCH2_Pin */
  GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nSLEEP_M2_Pin */
  GPIO_InitStruct.Pin = nSLEEP_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nSLEEP_M2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_DC_M2_Pin DIR_DC_M1_Pin */
  GPIO_InitStruct.Pin = DIR_DC_M2_Pin|DIR_DC_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FAULTn_M2_Pin SNSOUT_M2_Pin FAULTn_M1_Pin */
  GPIO_InitStruct.Pin = FAULTn_M2_Pin|SNSOUT_M2_Pin|FAULTn_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : nSLEEP_M1_Pin */
  GPIO_InitStruct.Pin = nSLEEP_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nSLEEP_M1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SNSOUT__M1_Pin */
  GPIO_InitStruct.Pin = SNSOUT__M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SNSOUT__M1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_EN_Pin */
  GPIO_InitStruct.Pin = MC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MC_EN_GPIO_Port, &GPIO_InitStruct);

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

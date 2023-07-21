
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define Sensor1	0xD0

uint8_t envio[15];
uint8_t receber[2];
float ax, ay, az, gx, gy, gz, temp, temperatura;
float cal_g_x, cal_g_y, cal_g_z;
float cal_a_x, cal_a_y, cal_a_z;
double ang_pitch_med, ang_roll_med;
double ac_total, ac_total_vetor[16], ac_av;
double ang_pitch_input, ang_roll_input, ang_yaw_input;
double ang_pitch, ang_roll, ang_yaw, cal_ang_pitch, cal_ang_roll;
double ang_pitch_ac, ang_roll_ac, giro_pitch, giro_roll;
double angleYZ, angleXZ;
double ang_pitch_acc, ang_roll_acc;
float cal_x = 0, cal_y = 0, cal_z = 0;
double angle_pitch_gyro, angle_roll_gyro;
bool first = false;
bool medir = true;
bool ligar = false;
uint32_t input_capture1 = 0;
uint32_t input_capture2 = 0;
uint32_t input_capture3 = 0;
uint32_t input_capture4 = 0;
uint32_t input_capture5 = 0;
uint32_t input_capture6 = 0;
volatile uint32_t canal_1 = 1000;
volatile uint32_t canal_2 = 1000;
volatile uint32_t canal_3 = 1000;
volatile uint32_t canal_4 = 1000;
volatile uint32_t canal_5 = 1000;
volatile uint32_t canal_6 = 1000;
uint32_t esc_1, esc_2, esc_3, esc_4;
uint32_t time = 0;
float vibra, vibra_total, vibra_time, v_bat = 0;
int i = 0, cal = 0, j = 1;
int teste, adc_time = 0;
unsigned int adc = 0;
int16_t rc_roll, rc_pitch, rc_yaw;
int16_t velocidade;

//Ganho PID
// Controle por velocidades angulares
float KP_roll = 0;			//1			4
float KI_roll = 0;		//0.02		0.01
float KD_roll = 0;			//15	5	25
#define PID_max_roll 300

float KP_pitch = 0;
float KI_pitch = 0;
float KD_pitch = 0;
#define PID_max_pitch 300

#define KP_yaw 0
#define KI_yaw 0
#define KD_yaw 0
#define PID_max_yaw 200

//PID Z - Controle de altura
#define KP_Z 1
#define KI_Z 0.02
#define PID_max_z 200

// Parámetros PID
int16_t erro_roll, erro_pitch, erro_yaw, erro_roll_anterior, vel_z, acc_total;
int16_t erro_pitch_anterior, erro_yaw_anterior, erro_z,erro_z_anterior;
float roll_P, roll_I, roll_D, pitch_P, pitch_I, pitch_D, yaw_P, yaw_I, yaw_D, z_P, z_I;
int16_t PID_roll, PID_pitch, PID_yaw, PID_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void calibrar();
void ligado();
void dados_sensor();
void max_min();
void roll_pitch_yaw();
void PID();
void PID_altitude();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //Descobrir o endereço do sensor
  /*for(i=0; i < 255; i++)
  {
	  if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK)
	  {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  break;
	  }
  }*/

  //Ativa o sensor
  envio[0] = 0x6B;
  envio[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)Sensor1, envio, 2, 50);
  //Configura acelerômetro em 8g
  envio[0] = 0x1C;
  envio[1] = 0x10;
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)Sensor1, envio, 2, 50);
  //Configura giroscópio em 1000 dps
  envio[0] = 0x1B;
  envio[1] = 0x08;
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)Sensor1, envio, 2, 50);

  //Inicia as interrupções do Input Capture
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

  //Inicia interrupção global do timer 7
  HAL_TIM_Base_Start_IT(&htim7);

  //Inicia os PWMs
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  HAL_Delay(3000); //Tempo estabilizar o quad

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  calibrar();
	  ligado();
	  if(ligar == true)
	  {
		  //Condição pegar dados do sensor (4ms) em ângulos
		  if((uwTick) >= (time))
		  {
			  dados_sensor();
			  i++;
			  time = uwTick+4;
			  //Converte os canais em graus
			  roll_pitch_yaw();
			  //Ação do PID
			  PID();
			  //Ação PID altitude
			  if(canal_3 > 1450 && rc_roll == 0 && rc_pitch == 0)
			  {
				  PID_altitude();
			  }

			  //Ação de controle
			  esc_1 = (velocidade-PID_z) - PID_pitch + PID_roll - PID_yaw;
			  esc_2 = (velocidade-PID_z) + PID_pitch + PID_roll + PID_yaw;
			  esc_3 = (velocidade-PID_z) + PID_pitch - PID_roll - PID_yaw;
			  esc_4 = (velocidade-PID_z) - PID_pitch - PID_roll + PID_yaw;

			  //Aceleração em relação a bateria
			  if (v_bat < 12 && v_bat > 10)
			  {
				  esc_1 = esc_1 + esc_1 * ((12.6 - v_bat)/(float)30);
				  esc_2 = esc_2 + esc_2 * ((12.6 - v_bat)/(float)30);
				  esc_3 = esc_3 + esc_3 * ((12.6 - v_bat)/(float)30);
				  esc_4 = esc_4 + esc_4 * ((12.6 - v_bat)/(float)30);
			  }
			  max_min();
			  i = 0;

			  //Aplica PID no motores
			  //Motor 2
			  htim8.Instance->CCR1 = esc_2;
			  //Motor 1
			  htim8.Instance->CCR2 = esc_1;
			  //Motor 3
			  htim8.Instance->CCR3 = esc_3;
			  //Motor 4
			  htim8.Instance->CCR4 = esc_4;

			  adc_time++;
			  //Leitura ADC a cada 1 segundo
			  if(adc_time >= 250)
			  {
				  //Inicia o ADC
				  HAL_ADC_Start(&hadc1);
				  //Converte a tensão
				  HAL_ADC_PollForConversion(&hadc1, 100);
				  //Armazena valor na variavel
				  adc = HAL_ADC_GetValue(&hadc1);
			  	  //Para o ADC
				  HAL_ADC_Stop(&hadc1);
				  //Transforma valor adc em tensão bateria
				  v_bat = (12.6*adc)/3590;
				  adc_time = 0;
			  	  //Condição acender LED - baixa bateria
				  if(v_bat <= 11 && v_bat >= 9)
				  {
					  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				  }
			  }
		  }
	  }
	  else
	  {
		  //Motores parados
		  htim8.Instance->CCR1 = canal_3;
		  htim8.Instance->CCR2 = canal_3;
		  htim8.Instance->CCR3 = canal_3;
		  htim8.Instance->CCR4 = canal_3;
	  	  //Desligado zera todos os valores PID
		  PID_roll = 0;
		  roll_I = 0;
		  erro_roll_anterior = 0;
		  PID_pitch = 0;
		  pitch_I = 0;
		  erro_pitch_anterior = 0;
		  PID_yaw = 0;
		  yaw_I = 0;
		  erro_yaw_anterior = 0;
	  }

	  //Vibração
	  /*dados_sensor();
	  vibra = ac_total + vibra;
	  if((vibra_time+4) <= uwTick)
	  {
		  vibra_total = vibra/(75*j);
		  j = 1;
		  vibra = 0;
	  }
	  j++;*/

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 20000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 50000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 79;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 50000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance==TIM2)
  {
	input_capture1 = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);    //Lê valor do canal 1 - TIM2
	__HAL_TIM_SetCounter(&htim2, 0);    //redefinir contador após a interrupção da captura de entrada
  }
  if (input_capture1 <= 2050)
  {
	  canal_1 = input_capture1;
	  if(canal_1 <= 1000)
	  {
		  canal_1 = 1000;
	  }
  }

  if (htim->Instance==TIM3)
    {
  	input_capture2 = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_3);    //Lê valor do canal 2 - TIM3
  	__HAL_TIM_SetCounter(&htim3, 0);    //redefinir contador após a interrupção da captura de entrada
    }
  if (input_capture2 <= 2050)
    {
  	  canal_2 = input_capture2;
  	  if(canal_2 <= 1000)
  	  {
  		  canal_2 = 1000;
  	  }
    }

  if (htim->Instance==TIM4)
    {
  	input_capture3 = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_3);    //Lê valor do canal 3 - TIM4
  	__HAL_TIM_SetCounter(&htim4, 0);    //redefinir contador após a interrupção da captura de entrada
    }
  if (input_capture3 <= 2050)
    {
  	  canal_3 = input_capture3;
  	  if(canal_3 <= 1000)
  	  {
  		  canal_3 = 1000;
	  }
    }

  if (htim->Instance==TIM5)
    {
  	input_capture4 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_2);    //Lê valor do canal 4 - TIM5
  	__HAL_TIM_SetCounter(&htim5, 0);    //redefinir contador após a interrupção da captura de entrada
    }
  if (input_capture4 <= 2050)
    {
  	  canal_4 = input_capture4;
  	  if(canal_4 <= 1000)
  	  {
  		  canal_4 = 1000;
  	  }
    }

  if (htim->Instance==TIM16)
    {
  	input_capture5 = __HAL_TIM_GetCompare(&htim16, TIM_CHANNEL_1);    //Lê valor do canal 4 - TIM5
  	__HAL_TIM_SetCounter(&htim16, 0);    //redefinir contador após a interrupção da captura de entrada
    }
  if (input_capture5 <= 2050)
    {
  	  canal_5 = input_capture5;
    }

  if (htim->Instance==TIM17)
    {
  	input_capture6 = __HAL_TIM_GetCompare(&htim17, TIM_CHANNEL_1);    //Lê valor do canal 4 - TIM5
  	__HAL_TIM_SetCounter(&htim17, 0);    //redefinir contador após a interrupção da captura de entrada
    }
  if (input_capture6 <= 2050)
    {
  	  canal_6 = input_capture6;
    }

}

void max_min()
{
	if(esc_1 <= 1000)
	{
		esc_1 = 1000;
	}
	else if(esc_1 >= 2000)
	{
		esc_1 = 2000;
	}
	if(esc_2 <= 1000)
	{
		esc_2 = 1000;
	}
	else if(esc_2 >= 2000)
	{
		esc_2 = 2000;
	}
	if(esc_3 <= 1000)
	{
		esc_3 = 1000;
	}
	else if(esc_3 >= 2000)
	{
		esc_3 = 2000;
	}
	if(esc_4 <= 1000)
	{
		esc_4 = 1000;
	}
	else if(esc_4 >= 2000)
	{
		esc_4 = 2000;
	}
}


void dados_sensor()
{
  	envio[0] = 0x3B;
  	envio[1] = 0x00;
  	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)Sensor1, &envio[0], 1, 50);
  	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)Sensor1, receber, 14, 50);
  	//Pega os dados do sensor
  	ax = (float)(((int16_t)(receber[0] << 8 | receber[1])));
  	ay = (float)(((int16_t)(receber[2] << 8 | receber[3])));
  	az = (float)(((int16_t)(receber[4] << 8 | receber[5])));
  	temp = (float)(((int16_t)(receber[6] << 8 | receber[7])));
  	gx = (float)(((int16_t)(receber[8] << 8 | receber[9])));
  	gy = (float)(((int16_t)(receber[10] << 8 | receber[11])));
  	gz = (float)(((int16_t)(receber[12] << 8 | receber[13])));

	//Offset dos ângulos
	gx = gx - cal_g_x;
	gy = gy - cal_g_y;
	gz = gz - cal_g_z;

	//Filtro suavizar a resposta, dando mais importância ao valor anterior
	ang_pitch_input = (double)((ang_pitch_input * 0.7) + ((gx/65.5) * 0.3));
	ang_roll_input = (double)((ang_roll_input * 0.7) + ((gy/65.5) * 0.3));
	ang_yaw_input = (double)((ang_yaw_input * 0.7) + ((gz/65.5) * 0.3));

	//0.0000611 = 1 / (250Hz / 65.5)
	ang_pitch += gx * 0.0000611;
	ang_roll += gy * 0.0000611;

	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
	ang_pitch -= ang_roll * sin(gz * 0.000001066);
	ang_roll += ang_pitch * sin(gz * 0.000001066);

	ac_total = sqrt((ax*ax)+(ay*ay)+(az*az));;

	if(abs(ay) < ac_total){
		ang_pitch_acc = asin((float)ay/ac_total)* 57.296;
	}
	if(abs(ax) < ac_total){
	    ang_roll_acc = asin((float)ax/ac_total)* -57.296;
	}

	if(first)
	{
		ang_pitch = ang_pitch * 0.9996 + ang_pitch_acc * 0.0004;
		ang_roll = ang_roll * 0.9996 + ang_roll_acc * 0.0004;
	}
	else
	{
		ang_pitch = ang_pitch_acc;
		ang_roll = ang_roll_acc;
		first = true;
	}

	if(ac_total<4050 && ac_total>3870)
	{
		ac_total = 4026;
		vel_z = 0;
	}
	//0.000973671 = (1 / 4026) * 9.8 * 100 (m->cm) * (1 / 250 Hz)
	vel_z = vel_z + ((ac_total-4026) * 0.000973671);

	ang_pitch = ang_pitch * 0.9996 + ang_pitch_acc * 0.0004;
	ang_roll = ang_roll * 0.9996 + ang_roll_acc * 0.0004;

 }


void ligado()
{
 	if(canal_6 > 1300)
 	{
  		ligar = true;
  	}
  	else
  	{
  		ligar = false;
  		time = uwTick+4;
  	}
}

void roll_pitch_yaw()
{
	//Converte canal 2 do rádio em graus - PID_roll
	if(canal_2 > 1512){
		rc_roll = (canal_2 - 1512)/3.05;  //(500-12)/3.05 = 160 graus/s (máximo)
	}
	else if(canal_2 < 1488){
		rc_roll = ((1488 - canal_2)/3.05)*(-1);
	}
	else if(canal_2 >= 1488 && canal_2 <= 1512){
		rc_roll = 0;
	}
	////Converte canal 1 do rádio em graus - PID_pitch
	if(canal_1 > 1512){
		rc_pitch = (canal_1 - 1512)/3.05;
	}
	else if(canal_1 < 1488){
		rc_pitch = ((1488 - canal_1)/3.05)*(-1);
	}
	else if(canal_1 >= 1488 && canal_1 <= 1512){
		rc_pitch = 0;
	}
	////Converte canal 4 do rádio em graus - PID_yaw
	if(canal_4 > 1512){
		rc_yaw = (canal_4 - 1512)/3.05;
	}
	else if(canal_4 < 1488){
		rc_yaw = ((1488 - canal_4)/3.05)*(-1);
	}
	else if(canal_4 >= 1488 && canal_4 <= 1512){
		rc_yaw = 0;
	}
	//Velocidade de aceleração
	velocidade = canal_3;
	if (velocidade>1900){
		velocidade = 1900;
	}
}

void calibrar()
{
	if(medir == true)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		for(cal = 0; cal < 2000; cal++)
		{
			envio[0] = 0x3B;
			envio[1] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)Sensor1, &envio[0], 1, 50);

			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)Sensor1, receber, 14, 50);

			gx = (float)(((int16_t)(receber[8] << 8 | receber[9])));
			gy = (float)(((int16_t)(receber[10] << 8 | receber[11])));
			gz = (float)(((int16_t)(receber[12] << 8 | receber[13])));

			cal_g_x += gx;
			cal_g_y += gy;
			cal_g_z += gz;

		}

		cal_g_x = cal_g_x/2000;
		cal_g_y = cal_g_y/2000;
		cal_g_z = cal_g_z/2000;

		medir = false;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	}
}

void PID()
{
	//Cálculo PID eixo X (roll)
	//Cálculo do erro
	erro_roll = ang_roll_input - rc_roll;
	//Cálculo ação proporcional
	roll_P = erro_roll * KP_roll;
	//Cálculo ação integral
	roll_I = roll_I + (erro_roll * KI_roll);
	//A ação integral aumenta muito rápido, deve ser limitada
	if(roll_I>PID_max_roll)roll_I = PID_max_roll;
	else if(roll_I<(PID_max_roll*(-1)))roll_I = PID_max_roll*(-1);
	//Cálculo ação derivativo
	roll_D = (erro_roll - erro_roll_anterior) * KD_roll;
	//Soma ganhos para obter o PID
	PID_roll = roll_P + roll_I + roll_D;
	//Limitação do PID
	if(PID_roll>PID_max_roll)PID_roll = PID_max_roll;
	else if(PID_roll<(PID_max_roll*(-1)))PID_roll = PID_max_roll*(-1);
	//Armazena o erro atual
	erro_roll_anterior = erro_roll;

	//Cálculo PID eixo Y (pith)
	//Cálculo do erro
	erro_pitch = ang_pitch_input - rc_pitch;
	//Cálculo ação proporcional
	pitch_P = erro_pitch * KP_pitch;
	//Cálculo ação integral
	pitch_I = pitch_I + (erro_pitch * KI_pitch);
	//A ação integral aumenta muito rapido, deve ser limitada
	if(pitch_I>PID_max_pitch)pitch_I = PID_max_pitch;
	else if(pitch_I<(PID_max_pitch*(-1)))pitch_I = PID_max_pitch*(-1);
	//Calculo ação derivativa
	pitch_D = (erro_pitch - erro_pitch_anterior) * KD_pitch;
	//Soma ganhos para obter o PID
	PID_pitch = pitch_P + pitch_I + pitch_D;
	//Limitação do PID
	if(PID_pitch>PID_max_pitch)PID_pitch = PID_max_pitch;
	else if(PID_pitch<(PID_max_pitch*(-1)))PID_pitch = PID_max_pitch*(-1);
	//Armazena o erro atual
	erro_pitch_anterior = erro_pitch;

	//Cálculo PID eixo Y (yaw)
	//Calculo do erro
	erro_yaw = ang_yaw_input - rc_yaw;
	//Calculo ação proporcional
	yaw_P = erro_yaw * KP_yaw;
	//Calculo ação integral
	yaw_I = yaw_I + (erro_yaw * KI_yaw);
	//A ação integral aumenta muito rapido, deve ser limitada
	if(yaw_I>PID_max_yaw)yaw_I = PID_max_yaw;
	else if(yaw_I<(PID_max_yaw*(-1)))yaw_I = PID_max_yaw*(-1);
	//Calculo ação derivativo
	yaw_D = (erro_yaw - erro_yaw_anterior) * KD_yaw;
	//Soma ganhos para obter o PID
	PID_yaw = yaw_P + yaw_I + yaw_D;
	//Limitação do PID
	if(PID_yaw>PID_max_yaw)PID_yaw = PID_max_yaw;
	else if(PID_yaw<(PID_max_yaw*(-1)))PID_yaw = PID_max_yaw*(-1);
	//Armazena o erro atual
	erro_yaw_anterior = erro_yaw;
}

void PID_altitude()
{
	//Erro velocidade
	erro_z = vel_z;
	//Ação proporcional
	z_P = erro_yaw * KP_Z;
	//Ação integral
	z_I = yaw_I + (erro_yaw * KI_Z);
	//Ação de controle
	PID_z = z_P + z_I;
	//Limitação do controle
	if(PID_z>PID_max_z)PID_z = PID_max_z;
	else if(PID_z<(PID_max_z*(-1)))PID_z = PID_max_z*(-1);
	//Erro anterior igual erro atual
	erro_z_anterior = erro_z;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

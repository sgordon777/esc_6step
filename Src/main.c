/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef __HAL_TIM_SET_TRGO
#define __HAL_TIM_SET_TRGO(__HANDLE__, __TRGO__) \
  MODIFY_REG((__HANDLE__)->Instance->CR2, TIM_CR2_MMS, (__TRGO__))
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
// throttle
//
// BUG: low RPM has weird backward-looking phases, cannot commutate correctly
// BUG: Blown cycle budget on commutating
// TODO: fix RPM (variable frequency)
// TODO: try 60K
// TODO: fit PWMHIGH mode
// TODO: Analog throttle
// TODO: try highres timer
// TODO: lookup table for commutator step switchblock
// TODO: rewrite time-critical bits in LL
// TODO: Make time ch4 handling a one-time, remove from commutation func
// TODO: Macros for trigger, tasks


#define ADC_BUF_SZ (2048)
#define TRG_BUF_SZ (2048)
#define TRIG_IDLE (0)
#define TRIG_PENDING (1)
#define TRIG_COMPLETE (2)
const int STATE_MEASURE = 0;
const int STATE_SPINUP = 1;
const int STATE_COMMUTATE = 2;

// MACRO
#define LPF(x, y, A, B) ( x*A + y*B )
#define TRIGGER_START(x)    (x = (x==TRIG_IDLE) ? TRIG_PENDING : x     )
#define TRIGGER_COMPLETE(x) (x = (x==TRIG_PENDING) ? TRIG_COMPLETE : x )
#define TRIGGER_RESET(x)    (x = TRIG_IDLE)
#define TRIGGER_STAT(x) (x)
const float CPU_CLK = 170E6;
void commutator(int step, uint16_t duty_cycle_ticks);
int16_t adc_buf[ADC_BUF_SZ];
uint32_t trg_buf[TRG_BUF_SZ];



// param
const int MOD_FREQ = 30000;
uint16_t per = (int)( CPU_CLK / (2*MOD_FREQ) );
// speed tables
#define INIT_SPEED (0.65)
float speed_vals[] = {0.60, 0.65, 0.77, 0.99};
int NUM_SPEEDS = 4;
int speed_ndx = 1; // 0=

// commutation tables
#if 1
// canocial (ccw)
uint32_t comm_source_chan[6] = 	{TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_3};
uint32_t comm_sink_chan[6] =   	{TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_2};
uint32_t comm_float_chan[6] =  	{TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1};
uint32_t comm_adc_chan[6] = 	{ADC_CHANNEL_11, ADC_CHANNEL_2, ADC_CHANNEL_1, ADC_CHANNEL_11, ADC_CHANNEL_2, ADC_CHANNEL_1};
uint32_t comm_zz_pol[6] =       {0, 1, 0, 1, 0, 1};
#else
// alternate (cw)
uint32_t comm_source_chan[6] = 	{TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_1};
uint32_t comm_sink_chan[6] =   	{TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_3};
uint32_t comm_float_chan[6] =  	{TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_2};
uint32_t comm_adc_chan[6] = 	{ADC_CHANNEL_11, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_11, ADC_CHANNEL_1, ADC_CHANNEL_2};
uint32_t comm_zz_pol[6] =       {1, 0, 1, 0, 1, 0};
#endif

#define COMM_MODE_PWMCOMP
//#define COMM_MODE_PWMHIGH
//#define COMM_MODE_PWMLOW
float frq_start = 10;  // hz
float frq_stop = 300;  // hz
float frq_inc = 200; // hz/sec
int hold_end_spinup = 0;
int motor_pp = 7;
float motor_res = 10;
float motor_inductino = 1E-5;
uint32_t dt = 1;
float ALFA_RPM = 0.003;
float ALFA_RPM1 = 0.997;


// global
int adc_ndx = 0;
int trg_ndx = 0;
int comm_step = 0;
int enable = 0;
float frq_commtask = 10;
int lim_commtask = 3000;  // MOD_FREQ / comm_freq
int comm_state = STATE_MEASURE; // 0 = measure, 1=windup, 2=commmutation
int zc_pol = 0;
int switched  = 0;
int comm_stabilize_count = 0;
float rpm_est = 0;
uint16_t ref;
uint32_t trig_measure = 0;
const uint32_t MEASURE_THRESH = ADC_BUF_SZ / 2;
float ttf;
uint32_t mod_tick = 0;
uint32_t delta_mod = 0;
uint32_t zc_sw = 0;
uint32_t bemf_sw = 0;
int32_t bemfdiff_sw = 0;
uint32_t comm_duty;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SetInjectedBEMFChannel(uint32_t adc_channel);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static int ctr_commtask = 0;

	if (hadc->Instance == ADC1)
    {
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		static uint16_t last_bemf = 0, bemf = 0;
        uint16_t bemf_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

        //bemf = (bemf_raw + last_bemf) / 2;
        bemf = bemf_raw;



        adc_buf[adc_ndx] = bemf;

        if (comm_state == STATE_MEASURE)
        {
        	// one-time state to measure ref voltage before motor running
        	if (adc_ndx >= MEASURE_THRESH) TRIGGER_START(trig_measure);   // schedule ref measurement
        }
        else if (comm_state == STATE_SPINUP)
        {
        	// manage spin-up state
			if (ctr_commtask >= lim_commtask)
			{
				commutator(comm_step, comm_duty);
				++comm_step;
				if (comm_step == 6) comm_step = 0;
				ctr_commtask = 0;
			}
			++ctr_commtask;
        }
        else if (comm_state == STATE_COMMUTATE)
        {
            // Manage commutating state: detect ZC and commutate
			if (    (comm_stabilize_count > 2) && ( (zc_pol == 0 && bemf < ref) || (zc_pol == 1 && bemf > ref) ) )
			{
				// zero cross detected. Commutate the motor.
				static uint32_t last_tick = 0;
				static uint32_t last_t = 0;
				static float y1 = 0;

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);				// run commutator
				commutator(comm_step, comm_duty);
				++comm_step;
				if (comm_step == 6) comm_step = 0;

				// log time, calculate RPM
				uint32_t t = DWT->CYCCNT;
				dt = (t - last_t);
				ttf = LPF((float)dt, y1, ALFA_RPM, ALFA_RPM1);
				y1 = ttf;

				last_t = t;
				trg_buf[trg_ndx] = dt;
				++trg_ndx;
				if (trg_ndx >= TRG_BUF_SZ) trg_ndx = 0;

				delta_mod = mod_tick - last_tick;
				bemf_sw = bemf;
				zc_sw = zc_pol;
				bemfdiff_sw = (int32_t)bemf - (int32_t)last_bemf;
				last_tick = mod_tick;
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);				// run commutator

        	}
        }

        ++adc_ndx;
        if (adc_ndx >= ADC_BUF_SZ) adc_ndx = 0;
        ++mod_tick;
        ++comm_stabilize_count;
        last_bemf = bemf_raw;
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

    }
}


void commutator(int step, uint16_t duty)
{
	//printf("phs=%d, dut=%d\n", step, duty);
    TIM_OC_InitTypeDef sConfig = {
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };

    // Determine active channels
    uint32_t source_ch = comm_source_chan[step];
    uint32_t sink_ch = comm_sink_chan[step];
    uint32_t float_ch = comm_float_chan[step];
    uint32_t adc_ch = comm_adc_chan[step];
    zc_pol = comm_zz_pol[step];

	SetInjectedBEMFChannel(adc_ch);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);											 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, per/2);

// SOURCE SIDE
    sConfig.OCMode = TIM_OCMODE_PWM1;
#if defined(COMM_MODE_PWMLOW)
    // Configure and enable SINK (low-side PWM) to be active (no PWM)
    sConfig.Pulse = TIM1->ARR;  // max duty
#else // COMM_MODE_PWMCOMP
    // Configure and enable SOURCE (high-side PWM)
    sConfig.Pulse = duty;
#endif
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, source_ch);
    HAL_TIM_PWM_Start(&htim1, source_ch);
    HAL_TIMEx_PWMN_Start(&htim1, source_ch);

// SINK SIDE
    sConfig.OCMode = TIM_OCMODE_PWM2;
#if defined(COMM_MODE_PWMHIGH)
    sConfig.Pulse = TIM1->ARR;  // max duty
#else // COMM_MODE_PWMCOMP
    sConfig.Pulse = duty;
#endif
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, sink_ch);
    HAL_TIM_PWM_Start(&htim1, sink_ch);
    HAL_TIMEx_PWMN_Start(&htim1, sink_ch);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    comm_stabilize_count = 0;
}

void SetInjectedBEMFChannel(uint32_t adc_channel)
{
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    sConfigInjected.InjectedChannel = adc_channel;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;

    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode = DISABLE;

    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        // This gets called when PA0 has an edge event
        // Your interrupt logic goes here
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // e.g., toggle an LED

        speed_ndx += 1;
        if (speed_ndx >= NUM_SPEEDS)
			speed_ndx = 0;
        comm_duty = per * speed_vals[speed_ndx];

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

	// Set DAC1_OUT1 to midpoint voltage (VBUS / 2 ≈ 1.65V if VBUS = 3.3V).




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
  MX_DAC1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(50);

  // enable ADC interrupt
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  __HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_JEOC);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);


  // enable GPIO interrupts
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // enable
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

  comm_duty = per * INIT_SPEED ; // out of TIM1->ARR


  commutator(comm_step, comm_duty);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t TASK_HANDLER_LIM = CPU_CLK / 100;
  uint32_t TASK_HANDLER_EL ;
  uint32_t TASK_HANDLER_t0 = DWT->CYCCNT ;

  uint32_t DIAG_HANDLER_LIM = CPU_CLK / 1;
  uint32_t DIAG_HANDLER_EL ;
  uint32_t DIAG_HANDLER_t0 = DWT->CYCCNT ;

  frq_commtask = frq_start;
  lim_commtask = MOD_FREQ / frq_commtask;
  // set ref channel for startup
  SetInjectedBEMFChannel(ADC_CHANNEL_14);

  while (1)
  {

	  // spinup handler
	  TASK_HANDLER_EL = DWT->CYCCNT - TASK_HANDLER_t0;
	  if (TASK_HANDLER_EL >= TASK_HANDLER_LIM)
	  {
		  if (comm_state == STATE_MEASURE)
		  {
			  // check measure trigger
			  if (TRIGGER_STAT(trig_measure) == TRIG_PENDING)
			  {
				  float x=0;
				  float one_over_sz = 1.0 / MEASURE_THRESH;
				  for (int i=0; i<MEASURE_THRESH; i++ )
				  {
					  x = x + adc_buf[i] * one_over_sz;
					  //printf("ndx=%d, raw=%d val=%f\n", i, adc_buf[i] ,x);
				  }
				  ref = (int)x / 2;

				  TRIGGER_COMPLETE(trig_measure);
				  // start spinup
				  comm_state = STATE_SPINUP;
				  printf("transitioning from measure to spinup, vref=%f x=%u \n", x, ref);

			  }
		  }
		  else if (comm_state == STATE_SPINUP)
		  {
			  if (frq_commtask < frq_stop)
			  {
				  frq_commtask += frq_inc / 100.0;
				  lim_commtask = MOD_FREQ / frq_commtask;
			  }
			  else
			  {
				  if (comm_state == STATE_SPINUP && hold_end_spinup == 0)
				  {
					  comm_state = STATE_COMMUTATE;
					  printf("transitioning from spin-up to BEMF commutation\n");
				  }
			  }
		  }
		  else if (comm_state == STATE_COMMUTATE)
		  {
			  // compute RPM
			  rpm_est = 60.0 / ( (ttf/CPU_CLK) * 6 * motor_pp );
		  }

		  TASK_HANDLER_t0 = DWT->CYCCNT;
	  }


	  // spinup handler
	  DIAG_HANDLER_EL = DWT->CYCCNT - DIAG_HANDLER_t0;
	  if (DIAG_HANDLER_EL >= DIAG_HANDLER_LIM)
	  {
		  uint32_t maxk = (uint32_t)( (ttf / CPU_CLK) * MOD_FREQ );
		  printf("state=%d, spd=%d, RPM=%.0f, ref=%u, com_delt=%.1f, com_deltk=[%u/%u] com_bemfsw=%u, com_bemfdiff=%d, sw_zc=%u\n",
				  comm_state, speed_ndx, rpm_est, ref, ttf, delta_mod, maxk, bemf_sw, bemfdiff_sw, zc_sw);
		  DIAG_HANDLER_t0 = DWT->CYCCNT;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  // You can use TIM_TRGO_UPDATE for end-of-PWM-cycle trigger or TIM_TRGO_OCxREF for precise control if using a spare channel like TIM1_CH4.
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = per;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  htim1.Instance->CCR4 = per / 2;

  // Manually override the TIM1 TRGO trigger to use CH4 compare event
  __HAL_TIM_SET_TRGO(&htim1, TIM_TRGO_OC4REF);



  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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

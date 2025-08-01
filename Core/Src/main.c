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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SIGNAL_LENGTH       128
#define FILTER_TAP_NUM      29


#define COM_START_SEND_SIGNAL	0x10
#define COM_END_SEND_SIGNAL		0x11
#define ANS_MATLAB_READY_RX		0x81





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool FlagKey = false;
bool FalgReadyPacketRx = false;

float32_t inputSignal[SIGNAL_LENGTH];
float32_t outputSignal[SIGNAL_LENGTH];
float32_t firState[SIGNAL_LENGTH + FILTER_TAP_NUM - 1];
uint16_t  SamplingRate = 0;


/* FIR Filter Coefficients (Low-pass FIR filter with Fs=1kHz, Fc≈50Hz) -----*/
const float32_t firCoeffs[FILTER_TAP_NUM] = {
  -0.0018225, -0.0034014, -0.0053022, -0.0072276, -0.0086242, -0.0087205,
  -0.0066352, -0.0015381, 0.0071607, 0.0191279, 0.0334224, 0.0486695,
   0.0632314, 0.0754156, 0.0837471, 0.0871925, 0.0853609, 0.0785769,
   0.0677995, 0.0544983, 0.0403817, 0.0271259, 0.0161513, 0.0083847,
   0.0041585, 0.0031884, 0.0045681, 0.0070031, 0.0089389
};


arm_fir_instance_f32 firFilter;


uint8_t BufferSampleSignal_1[17] = {'\0'};
uint8_t BufferReceive[64];



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	FlagKey = true;
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* Signal Generator Function ------------------------------------------------*/
void generateSignal(float Fs)
{
  for (int i = 0; i < SIGNAL_LENGTH; i++) {
    float t = (float)i / Fs; // Ts = 1ms -> Fs = 1kHz
    inputSignal[i] = arm_sin_f32(2 * PI * 10 * t) + 0.5f * arm_sin_f32(2 * PI * 100 * t);
  }
}

/* FIR Filtering Function ---------------------------------------------------*/
void processFIR(void)
{
  arm_fir_init_f32(&firFilter, FILTER_TAP_NUM, (float32_t *)firCoeffs, firState, SIGNAL_LENGTH);
  arm_fir_f32(&firFilter, inputSignal, outputSignal, SIGNAL_LENGTH);
}



bool CreateBuffeSample(uint8_t * BufferSample, uint32_t indexSample, int32_t sample)
{

/*  | 0  |  1 |  2 |  3 |  4 |  5 ||    6    |    7    |    8    |    9    ||    10    |    11    |    12    |    13    ||      14     |      15     |
 *	|____|____|____|____|____|____||_________|_________|_________|_________||__________|__________|__________|__________||_____________|_____________|
 *	|	 |    |    |    |    |    ||         | 		   |         |		   ||		   |	      |		     |		    ||			   |			 |
 *	| AA | 55 | 5A | A5 | BB | TP || Data[3] | Data[2] | Data[1] | Data[0] || Count[3] | Count[2] | Count[1] | Count[0] || CheckSum[1] | CheckSum[0] |
 *	|____|____|____|____|____|____||_________|_________|_________|_________||__________|__________|__________|__________||_____________|_____________|
 *   ////////////Header\\\\\\\\\\\  ////////////////Sample\\\\\\\\\\\\\\\\\  ////////////////Index Sample\\\\\\\\\\\\\\\  ////////Check Sum\\\\\\\\\\
 */


	#define TYPE_PACKET		0x91
	uint16_t checkSum = 0;


	//[0] : header
	*BufferSample = 0xAA;
	checkSum += *BufferSample;
	BufferSample++;

	//[1] : header
	*BufferSample = 0x55;
	checkSum += *BufferSample;
	BufferSample++;

	//[2] : header
	*BufferSample = 0x5A;
	checkSum += *BufferSample;
	BufferSample++;

	//[3] : header
	*BufferSample = 0xA5;
	checkSum += *BufferSample;
	BufferSample++;

	//[4] : header
	*BufferSample = 0xBB;
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[5] : TYPE PACKET
	*BufferSample = TYPE_PACKET;
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[6] : Data[3]
	*BufferSample = (sample & 0xff000000) >> 24;
	checkSum += *BufferSample;
	BufferSample++;

	//[7] : Data[2]
	*BufferSample = (sample & 0x00ff0000) >> 16;
	checkSum += *BufferSample;
	BufferSample++;

	//[8] : Data[1]
	*BufferSample = (sample & 0x0000ff00) >> 8;
	checkSum += *BufferSample;
	BufferSample++;

	//[9] : Data[0]
	*BufferSample = (sample & 0x000000ff);
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[10] : Count[3]
	*BufferSample = (indexSample & 0xff000000) >> 24;
	checkSum += *BufferSample;
	BufferSample++;

	//[11] : Count[2]
	*BufferSample = (indexSample & 0x00ff0000) >> 16;
	checkSum += *BufferSample;
	BufferSample++;

	//[12] : Count[1]
	*BufferSample = (indexSample & 0x0000ff00) >> 8;
	checkSum += *BufferSample;
	BufferSample++;

	//[13] : Count[0]
	*BufferSample = (indexSample & 0x000000ff);
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[15] : CheckSum[0]
	*BufferSample = (checkSum & 0xff00) >> 8;
	BufferSample++;

	//[14] : CheckSum[1]
	*BufferSample = (checkSum & 0x00ff);
	//BufferSample++;



	return 1;
}




bool CreateCommand(uint8_t * command_buffer , uint8_t command_type,uint8_t scalePow, uint8_t numSignal, uint32_t numSample ,uint16_t SmaplRate)
{

/*  | 0  |  1 |  2 |  3 |  4  |  5 ||      6       |     7     |   8   |   9   ||      10      ||      11      ||      12      ||      13      ||      14      ||      15     |
 *	|____|____|____|____|_____|____||______________|___________|_______|_______||______________||______________||______________||______________||______________||_____________|
 *	|	 |    |    |    |scale|    ||         	   | 		   | SAMPL | SAMPL ||	   	       ||	           ||		       ||		       ||			   ||			  |
 *	| AA | 55 | 5A | A5 | Pow | TP || command_type | numSignal |RATE[1]|RATE[0]|| numSample[3] || numSample[2] || numSample[1] || numSample[0] || CheckSum[1]  || CheckSum[0] |
 *	|____|____|____|____|_____|____||______________|___________|_______|_______||______________||______________||______________||______________||______________||_____________|
 *   ////////////Header\\\\\\\\\\\\  ///////\\\\\\\ //////\\\\\ /// \\\ /// \\\  ////////////////////////Number Samples\\\\\\\\\\\\\\\\\\\\\\\  //////////Check Sum\\\\\\\\\\
 */



	#define TYPE_PACKET		0x15
	uint16_t checkSum = 0;



	//[0] : header
	*command_buffer = 0xAA;
	checkSum += *command_buffer;
	command_buffer++;

	//[1] : header
	*command_buffer = 0x55;
	checkSum += *command_buffer;
	command_buffer++;

	//[2] : header
	*command_buffer = 0x5A;
	checkSum += *command_buffer;
	command_buffer++;

	//[3] : header
	*command_buffer = 0xA5;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[4] : scale Power
	*command_buffer = 0xBB;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[5] : TYPE PACKET
	*command_buffer = TYPE_PACKET;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[6] : command_type
	*command_buffer = command_type;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[7] : numSignal
	*command_buffer = numSignal;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[8] : SampleRate[1]
	*command_buffer = (SmaplRate & 0xff00) >> 8;
	checkSum += *command_buffer;
	command_buffer++;


	//[9] : SampleRate[0]
	*command_buffer = (SmaplRate & 0x00ff);
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[10] : numSample[3]
	*command_buffer = (numSample & 0xff000000) >> 24;
	checkSum += *command_buffer;
	command_buffer++;

	//[11] : numSample[2]
	*command_buffer = (numSample & 0x00ff0000) >> 16;
	checkSum += *command_buffer;
	command_buffer++;

	//[12] : numSample[1]
	*command_buffer = (numSample & 0x0000ff00) >> 8;
	checkSum += *command_buffer;
	command_buffer++;

	//[13] : numSample[0]
	*command_buffer = (numSample & 0x000000ff);
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[14] : CheckSum[0]
	*command_buffer = (checkSum & 0xff00) >> 8;
	command_buffer++;

	//[15] : CheckSum[1]
	*command_buffer = (checkSum & 0x00ff);
	//BufferSample++;

	return 1;
}


bool AnalizeCommand(uint8_t analizePorpuse)
{
#define LENGTH_COMMAND	16

	uint16_t cal_checkSum = 0;
	uint16_t com_chechSum = 0;


	if(BufferReceive[0] != 0xAA  ||  BufferReceive[1] != 0x55  ||  BufferReceive[2] != 0x5A  ||
	   BufferReceive[3] != 0xA5  ||  BufferReceive[4] != 0xBB  ||  BufferReceive[5] != 0xCC)
		return 0;

	for(uint8_t index = 0 ; index < LENGTH_COMMAND-2 ; index++)
	{
		cal_checkSum += BufferReceive[index];
	}

	com_chechSum = ((((uint16_t)BufferReceive[14]) << 8) | ((uint16_t)BufferReceive[15]));

	if(cal_checkSum != com_chechSum)
		return 0;


	if(BufferReceive[6] == ANS_MATLAB_READY_RX)
		return 1;
	else
		return 0;

}







bool SendSignal(float32_t *Samples, int scale, uint8_t numSignal, uint32_t numSample, uint16_t sampleRaate)
{
	#define Limit_Wait		1000
	#define LENGTH_COMMAND	16
	#define ScalseSample	5

	int32_t	 sample_u32			= 0;
	uint32_t sampleIndex 	= 0;
	uint8_t  bufferTx[17] 	= {'\0'};
	uint16_t  timeWait 		= 0;


//HAL_Delay(5000);

	memset(bufferTx, '\0', 17);
	CreateCommand(bufferTx , COM_START_SEND_SIGNAL, ScalseSample, numSignal, numSample, sampleRaate);
	FalgReadyPacketRx = false;
	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
	timeWait = 0;
	//while(1);
	while(timeWait <= Limit_Wait)
	{
		HAL_Delay(1);
		timeWait++;
		if(FalgReadyPacketRx)
		{
			FalgReadyPacketRx = false;
			if(AnalizeCommand(ANS_MATLAB_READY_RX))
			{
				break;
			}
			else
				return 0;
		}
		if(timeWait == Limit_Wait)
			return 0;
	}


	for(sampleIndex = 0 ; sampleIndex < numSample ; sampleIndex++ , Samples++)
	{

		memset(bufferTx, '\0', LENGTH_COMMAND+1);
		sample_u32 = (int32_t)((*Samples) * scale);
		CreateBuffeSample(bufferTx, sampleIndex, sample_u32);
		CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
		HAL_Delay(2);
	}


	memset(bufferTx, '\0', LENGTH_COMMAND+1);
	CreateCommand(bufferTx , COM_END_SEND_SIGNAL, ScalseSample, numSignal, numSample, sampleRaate);
	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);

}










/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t counter = 10000;
uint8_t data[10] = {'\0'};
uint8_t BufferReceive[64];
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  generateSignal(1000.0f);
  processFIR();


//  SendSignal(inputSignal, 100000, 1, SIGNAL_LENGTH);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//  SendSignal(inputSignal, 100000, 1, SIGNAL_LENGTH);




	  if(FlagKey)
	  {
		  SendSignal(inputSignal, 100000, 1, SIGNAL_LENGTH, 1000);
		  HAL_Delay(200);
		  SendSignal(outputSignal, 100000, 2, SIGNAL_LENGTH, 1000);
//		  counter++;
//		  sprintf(data, "%u\r\n", counter);
//		  CDC_Transmit_FS((uint8_t*)data, strlen(data));
//		  HAL_Delay(500);
		  FlagKey = false;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

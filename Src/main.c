/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "ADXL355.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint32_t * buf;
	int head;
	int tail;
	int size;
} fifo_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECONDS 1000
#define BUFFER_SIZE 17 // note that it is size n-1
//#define DATA_SIZE 15 //for FIFO debugging
#define DELAY 1 //in seconds

#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330)) // <-- change this to according to your supply voltage

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
fifo_t fifoX; //fifo declaration
fifo_t fifoY; //fifo declaration
fifo_t fifoZ; //fifo declaration
uint32_t bufferX[BUFFER_SIZE]; //FIFO buffer declaration
uint32_t bufferY[BUFFER_SIZE]; //FIFO buffer declaration
uint32_t bufferZ[BUFFER_SIZE]; //FIFO buffer declaration

uint32_t *pBufferX = &bufferX[0]; //pointer declarations
uint32_t *pBufferY = &bufferY[0]; //pointer declarations
uint32_t *pBufferZ = &bufferZ[0]; //pointer declarations
fifo_t *pFifoX = &fifoX; //pointer declarations
fifo_t *pFifoY = &fifoY; //pointer declarations
fifo_t *pFifoZ = &fifoZ; //pointer declarations
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int stateManager(int oldState, int index, char currentChar);
int commandVerifier(char string[]);
void commandHandler(int command, char string[]);
void commandExecutor(int command, char substring[]);

void commandHelp(char substring[]);
void commandADXL355(char substring[]);
void commandSPI_read(char substring[]);
void commandSPI_write(char substring[]);
void commandInit(char substring[]);
void commandMeasure(char substring[]);

void verifyOrientationFIFO(void);
void floatToString(float num, int decimals, char * str, int strLength);
float rangeToScale(int range);

void fifo_init(fifo_t *f, uint32_t *buf, int size);
int fifo_read(fifo_t *f, void * buf, int nbytes);
int fifo_write(fifo_t *f, const void *buf, int nbytes);
float fifo_average(fifo_t *f);
void fifoDebugger(void);

void printTempST(void);
float computeTemperature(uint32_t measure);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */
	enum commands { non_command, help, ADXL355, SPI_read, SPI_write, init , measure, FIFO } command;
	enum state { begin, next, eol } currentState;

   	uint8_t startupMessage[] = "Hello! Please type below: \r\n\n";
	uint8_t newLine[] = "\r\n";

	int startupMessageLength = 29;
	int newLineLength = 2;

	char in[1];
	char buffer [256];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  fifo_init(pFifoX, pBufferX, BUFFER_SIZE); //initialization for x FIFO
  fifo_init(pFifoY, pBufferY, BUFFER_SIZE); //initialization for y FIFO
  fifo_init(pFifoZ, pBufferZ, BUFFER_SIZE); //initialization for z FIFO
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Transmit(&huart2, newLine, newLineLength, 100); //prints new line
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, startupMessage, startupMessageLength, 100); //defining status.. "type something"

  int j = 0; //index of char we are about to type
  currentState = begin;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch ( currentState ) { //state machine

	  case begin :
		  j = 0; //first index

		  do
			  status = HAL_UART_Receive(&huart2, (uint8_t *)in, 1, 1000);
		  while (status != HAL_OK); //keep looping till they type

		  buffer[j] = in[0]; //copy what they typed onto the buffer
	  	  buffer[j+1] = '\0';

	  	  HAL_UART_Transmit(&huart2, (uint8_t *)in, 1, 100); //prints right away
	  	  currentState = stateManager(begin, j, in[0]); //switch states

	  	  break;

	  case next :
		  j++; //increment as they're still going to type

		  do
			  status = HAL_UART_Receive(&huart2, (uint8_t *)in, 1, 1000);
		  while (status != HAL_OK); //keep looping till they type

		  buffer[j] = in[0]; //copy what they typed onto the buffer
	  	  buffer[j+1] = '\0'; //cap the end of the string

	  	  HAL_UART_Transmit(&huart2, (uint8_t *)in, 1, 100); //prints right away

	  	  if (buffer[j] == '\177' || buffer[j] == 0x08) { //backspace (not sure about the \177 key but it was necessary)
	  		  j--;
	  		  buffer[j] = '\0'; // new line
	  		  j--;
	  	  }

	  	  currentState = stateManager(next, j, in[0]);

	  	  break;

	  case eol : //end of line

		  command = commandVerifier(buffer); //checking to see if a valid command was inputed
		  if (command != non_command)  //if command typed is valid then send it to the commandHandler
			  commandHandler(command, buffer);
		  else
			  printf("\r\nInvalid input... type 'help' for valid inputs\r\n");

		  HAL_UART_Transmit(&huart2, (uint8_t *) newLine, newLineLength, 100); //prints new line
		  currentState = stateManager(eol, j, in[0]); //switches states
		  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float computeTemperature(uint32_t measure) {

	float temp130 = (float) *TEMP130_CAL_ADDR;
	float temp30 = (float) *TEMP30_CAL_ADDR;

	float temperature = (float)(130-30)/(temp130-temp30); //slope
	temperature = temperature*((float)measure-*TEMP30_CAL_ADDR);
	temperature += 30.0;
	return(temperature);
}

void printTempST(void) {
	//ADC->CCR |= ADC_CCR_TSVREFE;
	//ADC->CCR &= ~ADC_CCR_VBATE ;

	uint32_t g_ADCValue;
	//float g_ADCValueF;
	//char g_ADCValueS[6];

	if (HAL_ADC_PollForConversion(&hadc, 1000000) == HAL_OK) {
		g_ADCValue = HAL_ADC_GetValue(&hadc);
		//g_ADCValueF = computeTemperature(g_ADCValue);
		//floatToString(g_ADCValueF, 2, g_ADCValueS, 5);

		printf("MCU Temp (C):, %lu", g_ADCValue);
	}
}

//these two functions below allow us to use printf on the STM32 controller though stdio.h - do not modify
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart2, &*c, 1, 10);
	return ch;
}
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for(DataIdx= 0; DataIdx < len; DataIdx++)
		__io_putchar(*ptr++);
return len;
}

/*
 * @param : void
 *
 * This function is called before reading from the FIFO - it ensures that it is 'oriented correctly'
 * That is, that the user will commence by reading a value from the x register, then y and z respectively
 */
void verifyOrientationFIFO (void) {
	enum state { one , firstZero, secondZero, searching } currentState; //state machine

	int timeoutChecker = 0;
	uint8_t a1, a2, a3; //used to read from a register (ie: x has 20 bits, we read 8 at a time)
	currentState = searching;

	uint8_t txData = (0x11 << 1) | 1 ;  //setting up the MOSI byte

	while (1) {  //keep looping till FIFO is oriented on x

		int counter = 0;

		if (timeoutChecker > 30000) {
			//printf("\r\nError: Timed out when trying to orient the FIFO\r\n");
			return; //simple timeout checker, allows us to try arond 300 times before exiting
		} else
			timeoutChecker++;

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // CS ON

		HAL_SPI_Transmit (&hspi1, &txData, 1, 100);

		HAL_SPI_Receive (&hspi1, &a1, 1, 100); //could be x, y, or z - most sig digits
		HAL_SPI_Receive (&hspi1, &a2, 1, 100); //could be x, y, or z - middle sig digits
		HAL_SPI_Receive (&hspi1, &a3, 1, 100); //could be x, y, or z - last 2 bits refer to the x bit,

		a3 = a3 & 3 ; //only look at last 2 bits, bc it contains a tag that lets you know if it is an x bit

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // CS OFF

		switch ( currentState ) {

		    case one : //an x bit
			    if ( a3 == 0 )
				    currentState = firstZero;
			    break;

		    case firstZero : //right after the x bit, therefore y
			    if ( a3 == 0 )
				    currentState = secondZero;
			    break;

		    case secondZero : //right after the y bit, therefore z
		    	counter++;

		    	if (counter > 32)
		    		return; //returns so that we will read on x
		    	else {
		    		currentState = one;
		    		break;
		    	}

		    default :
		    	if ( a3 == 1 )
				    currentState = one;
			    break;
		}
	}
}
/*
 * @param num - the float number we will convert into a string
 * @param decimals - the number of decimals our number will have
 * @param *str - pointer to the string we are going to write into
 * @param strLength - the length of the string we are going to write into
 *
 * This function takes in a float value, and writes it into a string
 */
void floatToString(float num, int decimals, char * str, int strLength) {
	int decimalPart;
	int integerPart;

	integerPart = num;
	num = num - integerPart;

	int scale = 1;

	for ( int i = 0 ; i < decimals ; i++ )
		scale *= 10 ;

	decimalPart = num * scale;

	if ( num > 0 )
		sprintf(str, "%d.%d", integerPart, decimalPart);
	else if ( num < 0 ) {
		decimalPart *= -1;
		decimalPart /= 10;
		if (integerPart == 0 )
			sprintf(str, "-%d.%d", integerPart, decimalPart);
		else
			sprintf(str, "%d.%d", integerPart, decimalPart);
	} else if ( num == 0 ) {
		sprintf(str, "%d.", integerPart);
		for ( int i = 0 ; i < decimals ; i++)
			sprintf(str, "%s0", str);
	}

	while (strlen(str) < strLength)
		strcat(str, "0");
}
float rangeToScale(int range) {
	switch ( range ) { //set the scale based off of the current range

	case 2 :
		return 256000.0;

	case 4 :
		return 128000.0;

	case 8 :
		return 64000.0;

	default :
		return 256000.0;
	}
}

/*
 * @param : oldState - the state that the user was at before typing a character
 * @param : index - the number of keys that the user has typed in a line minus 1
 * @param : currentChar - the character the user inputed
 *
 * This function decides what state to send the user into after typing in a character
 */
int stateManager(int oldState, int index, char currentChar) {
	enum state { begin, next, eol };

	if (oldState == eol) //after pressing enter, one should start a new line
		return begin;
	if (index > 253) //cannot read more than 1 byte of data, left some room to work w/ just in case
		return eol;

	switch ( currentChar ) {
	case 0x08 : //backspace
		if (index == 0)
			return begin;
		else
			return next;

	case 0x20 : //space bar
		return next;

	case 0x0D : //enter
		return eol;

	case 0x0A : //line feed
		return eol;

	default : //any other key
		return next;
	}
}
/*
 * @param : string - the string of characters the user inputed, used to see if the user
 *                   inputed a valid command
 * @return : int - the command (enum) that was selected, or non_command if something else was typed
 *
 * This function was used to verify whether a valid command was inputed
 */
int commandVerifier(char string[]) {
	enum commands { non_command, help, ADXL355, SPI_read, SPI_write, init , measure, FIFO } ;

	char parser;
	int size = 0;
	do { //until it hits the space bar key or enter
		parser = string[size];
		size++;
	} while (parser != 0x20 && parser != 0x0D);

	size--;

	char commandString[size+1];
	for (int i = 0 ; i < size ; i++)
		commandString[i] = string[i];

	commandString[size] = '\0'; // cap off the first word as its own string

	if (strcmp(commandString, "help") == 0) //check to see if it matches any commands
		return help;
	else if (strcmp(commandString, "ADXL355") == 0)
		return ADXL355;
	else if (strcmp(commandString, "SPI_read") == 0)
			return SPI_read;
	else if (strcmp(commandString, "SPI_write") == 0)
			return SPI_write;
	else if (strcmp(commandString, "init") == 0)
		return init;
	else if (strcmp(commandString, "measure") == 0)
		return measure;
	else if (strcmp(commandString, "FIFO") == 0)
		return FIFO;
	else
		return non_command;
}
/*
 * @param : command - the command that has been verified
 * @param : string - what the user has typed in
 *
 * This function splits up the command string into the substring not containing the command itself,
 * and sends it to be executed
 */
void commandHandler(int command, char string[]) {
	enum commands { non_command, help, ADXL355, SPI_read, SPI_write, init , measure, FIFO };
	int size = -1; //garbage size value initialization

	switch ( command ) {
	case help :
		size = strlen(string)-strlen("help ");
		break;

	case ADXL355 :
		size = strlen(string)-strlen("ADXL355 ");
		break;

	case SPI_read :
		size = strlen(string)-strlen("SPI_read ");
		break;

	case SPI_write :
		size = strlen(string)- strlen("SPI_write ");
		break;

	case init :
		size = strlen(string)- strlen("init");
		break;

	case measure :
		size = strlen(string)- strlen("measure ");
		break;

	case FIFO :
		size = strlen(string)- strlen("FIFO");
		break;
	}

	char restOfLine[size];
	for (int i = 0 ; i < size ; i++)
		restOfLine[i] = string[strlen(string)-size+i];

	if (restOfLine[size-1] == 0x0D || restOfLine[size-1] == 0x20)
		restOfLine[size-1] = '\0'; //changed to size-1 instead of size
	else
		restOfLine[size] = '\0';

	if ((size == 0 && command == help) || (size == 0 && command == init) || (size == 0 && command == FIFO))
		commandExecutor(command, "\r");
	else
		commandExecutor(command, restOfLine);

}
/*
 * @param : command, substring - the command and substring read from a user
 *
 * This function calls the appropriate function corresponding to the command given
 */
void commandExecutor(int command, char substring[]) {
	enum commands { non_command, help, ADXL355, SPI_read, SPI_write, init , measure, FIFO };

	switch ( command ) {
	case help :
		commandHelp(substring);
		return;

	case ADXL355 :
		commandADXL355(substring);
		return;

	case SPI_read :
		commandSPI_read(substring);
		return;

	case SPI_write :
		commandSPI_write(substring);
		return;

	case init :
		commandInit(substring);
		return;

	case measure :
		commandMeasure(substring);
		return;

	case FIFO :
		fifoDebugger();
		return;
	}
}

/*
 * @param : substring
 *
 * This command prints some basic help to the user, so that they know how to use this software,
 * hand-in-hand with the ADXL355 eval board
 */
void commandHelp(char substring[]) {
	int size = strlen(substring);

	if (size == 1 && substring[0] == 0x0D) {
		printf("\r\nTo use a command, enter ADXL355, followed by the command/parameters, \r\nwith a single space in "
				"between each word.\r\n\nCurrent valid commands are: SPI_read , SPI_write , init , measure\r\n");
		printf("To check the parameters for a specific function, type \"help <function_name>\"\r\n");
		return;
	} else if ((strcmp(substring, "ADXL355 SPI_read") == 0) || (strcmp(substring, "SPI_read") == 0)) {
		printf("\r\nTo read from a register, type \"ADXL355 SPI_read <register_number>\"");
		printf("\r\nNote that registers are unsigned integers, in hexadecimal form.");
		printf("\r\n");
		return;
	} else if ((strcmp(substring, "ADXL355 SPI_write") == 0) || (strcmp(substring, "SPI_write") == 0)) {
		printf("\r\nTo write to a register, type \"ADXL355 SPI_write <register_number> <data_you_are_sending>\"");
		printf("\r\nNote that registers are unsigned integers, in hexadecimal form, while data is in decimal form.");
		printf("\r\n");
		return;
	} else if ((strcmp(substring, "ADXL355 measure") == 0) || (strcmp(substring, "measure") == 0)) {
		printf("\r\nTo measure all values, type \"ADXL355 measure ALL <delay time in seconds\"");
		printf("\r\n");
		return;
	} else if ((strcmp(substring, "ADXL355 init") == 0) || (strcmp(substring, "init") == 0)) {
		printf("\r\nTo reset and initialize the accelerometer, type \"ADXL355 init\"");
		printf("\r\n");
		return;
	} else {
		printf("\r\nInvalid input - either type \"help\" or \"help <function_name>\"");
		printf("\r\n");
	}
}
/*
 * @param : substring
 *
 * This special command focuses on the ADXL355, ensuring that this is the eval board being called
 * It takes the rest of the string (substring) and resends it to the command verifier to determine
 * if a specific ADXL355 command has been called
 */
void commandADXL355(char substring[]) {
	enum commands { non_command, help, ADXL355, SPI_read, SPI_write, init , measure, FIFO } command;

	command = commandVerifier(substring); //checking to see if a valid ADXL355 command was inputed
	if (command != non_command)  //if command typed is valid then send it to the commandHandler
		commandHandler(command, substring);
	else
		printf("\nError: Invalid command. Valid commands are start, stop,\r\nSPI_read, SPI_write, init , measure \r\n");
}
/*
 * @param : substring
 *
 * This function uses SPI to read from the ADXL355 accelerometer
 * This function can be used to read a specific address, the range, or all registers at once,
 * depending on the input
 */
void commandSPI_read(char substring[]) {
	char inputAddress[strlen(substring)];
	int invalidAddress = 0x2F;
	int address;
	uint32_t data;

	int i = 0;

	do { //parsing the inputed address into a string we can work with
		inputAddress[i]=substring[i];
		i++;
	} while (substring[i] != 0x0D && i<=6);

	inputAddress[i] = '\0';

	if (strcmp(inputAddress, "ALL") == 0) { //prints out the reading at all registers
		for (int j = 0; j < 0x2F ; j++) {
			data = ADXL355_SPI_Read(j);
			printf("\r\nAt address 0x%.2X, SPI reads: 0x%.2lX", j, data);
		}
		printf("\r\n");
	} else if (strcmp(inputAddress, "range") == 0) { //reads range
		uint32_t range = ADXL355_Read_Range();
		printf("\nRange is currently: Plus/minus %ld g\r\n", range);
	} else { //reads a specific address
		address = strtol(inputAddress, NULL, 16);

		if (address == invalidAddress) {
			printf("\r\nError: Cannot read from RESET address 0x2F.\r\n");
			return;
		}

		if (address >= 0 && address <= 255) {
			data = ADXL355_SPI_Read(address);
			printf("\r\nAt address 0x%.2X, SPI reads: 0x%.2lX\r\n", address, data);
		}
		else
			printf("\r\nError: Parameter must be a valid address, in\r\nhexadecimal form, from 0 to 0xFF\r\n");
	}
}
/*
 * @param : substring
 *
 * This function uses SPI to write to the ADXL355 accelerometer
 * This function can be used to write to a specific address, or the range (changing the range)
 * depending on the input
 */
void commandSPI_write(char substring[]) {

	char inputAddress[4];
	char inputMessage[4];

	int address;
	int message;

	int i = 0;

	do {
		inputAddress[i]=substring[i];
		i++;
	} while (substring[i] != 0x20 && i<=7);

	inputAddress[i] = '\0';

	if (strcmp(inputAddress, "range") == 0) { //writing to the range
		int j = 0;
		i++;

		do {
			inputMessage[j] = substring[i];
			i++;
			j++;
		} while (substring[i] != 0x0D && j <= 2);

		inputMessage[j] = '\0';

		message = strtol(inputMessage, NULL, 16);
		ADXL355_Set_Range (message);
		if (ADXL355_Read_Range() == message)
			printf("\nSuccess!\r\n");
		return;
	}

	address = strtol(inputAddress, NULL, 16);

	if (address >= 0x1E && address <= 0x2F) { //writing to a specific address
		int j = 0;
		i++;

		do {
			inputMessage[j] = substring[i];
			i++;
			j++;
		} while (substring[i] != 0x0D && j <= 4);

		inputMessage[j] = '\0';

		message = strtol(inputMessage, NULL, 16);

		if (message >= 0x00 && message <= 0xFF) {
			uint8_t old_message = ADXL355_SPI_Read(address);
			printf("\r\nAt address 0x%.2X: SPI has written 0x%.2X into 0x%.2X\r\n", address, old_message, message);
			ADXL355_SPI_Write(address, message, 1);
		}
		else
			printf("\r\nError: Invalid message, must be in hexadecimal form, from 0 to 0xFF.");
	}
	else
		printf("\r\nError: Parameter must be a valid address, in hexadecimal form, from 0 to 0xFF.\r\n"
				"You can only write to the registers ranging from 0x1E to 0x2F\r\n");
}
/*
 * @param : substring
 *
 * This function initializes the ADXL355 eval board, aka tests that we are reading correctly from
 * some important registers
 */
void commandInit(char substring[]) {
	if (substring[0] == 0x0D) {
		ADXL355_Init();
		commandSPI_write("0x28 9");
	} else
		printf("\nError: To initialize ADXL355, simply type 'ADXL355 init'\r\nfollowed by the enter key\r\n");
}
/*
 * @param : substring
 *
 * This command, when called to by typing 'ADXL355 measure ALL' will begin measuring values from the eval board,
 * until the user presses any key. It currently has the delay commented out, so it is outputting values at its
 * max speed, which is roughly 40 times per second. If one wishes, they can add a delay in the main loop, so
 * that it reads at a different frequency.
 * The values that this command measures are g, in the x,y,z directions, as well as temperature in celsius.
 */
void commandMeasure(char substring[]) {

	char command[3];
	char delayLength_s[4];
	int delay;
	int i = 0;
	float temperature;
	HAL_StatusTypeDef UARTstatus, SPIstatus;
	char in[1];
	float scale;
	uint16_t temp;
	uint8_t x1, x2, x3, y1, y2, y3, z1, z2, z3, temp1, temp2;
	uint32_t x0, y0, z0;
	float xf, yf, zf;
	int x, y, z;

	int boolIsEmpty;
	int boolIsMisalligned;

	char temperatureString[5];

	float averageX, averageY, averageZ;
	int index = 0;
	int terminate = 0;
	int boolWrote = 0;
	uint32_t dataOutX, dataOutY, dataOutZ;

	uint8_t txData = (0x11 << 1) | 1 ;

	do { //reading the first word, should be 'ALL'
		command[i]=substring[i];
		i++;
	} while (substring[i] != 0x20 && i<3); //3 or 4  ? ? ?

	command[i] = '\0';

	if (strcmp(command, "ALL") == 0) { //checking to see if the value inputted == "ALL"
		printf("\r\n");

		int j = 0;
		i++;

		do { //reading delay time
			delayLength_s[j] = substring[i];
			i++;
			j++;
		} while (substring[i] != 0x0D && j < 4);

		delayLength_s[j] = '\0';

		delay = strtol(delayLength_s, NULL, 10);

	} else {
		printf("\r\nError : enter 'ADXL355 measure ALL <delay_time>' to start measuring\r\n");
		return;
	}

	if ((delay > 0) && (delay < 10000))
		ADXL355_Start_Sensor(); //start the sensors
	else {
		printf("\r\nError : Delay time must be a positive integer from 1 to 9999 (in seconds)\r\n");
		return;
	}

	//printf("Temperature test 5 on Board #1 (com11)\r\nFIFO SIZE: 64 \r\nDELAY time: aiming for 1 min\r\n"
	//				"Filter setting 0x28 7\r\nTemperature chamber ascending in temp (roughly hourly)\r\n"); //new thing for testing

	uint32_t range = ADXL355_Read_Range(); //read the current range
	printf("Range is currently plus/minus %ld [g]\r\nPlease wait a moment for measurements to commence. . .\r\n", range); //print the range to the screen

	scale = rangeToScale(range); 	//set the scale based off of the current range

	verifyOrientationFIFO();

	int timeElapsed = 0;

	do { //main reading loop, only exits once the user presses a key on the keyboard

		boolIsMisalligned = 0;
		boolIsEmpty = 0;

		temp2 = ADXL355_SPI_Read(0x06); //reading from the temperature registers
		temp1 = ADXL355_SPI_Read(0x07);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // chip select ON

		HAL_SPI_Transmit (&hspi1, &txData, 1, 10);

		SPIstatus = HAL_SPI_Receive (&hspi1, &x3, 1, 10); //using FIFO to read the x,y,z registers
		SPIstatus = HAL_SPI_Receive (&hspi1, &x2, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &x1, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &y3, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &y2, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &y1, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &z3, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &z2, 1, 10);
		SPIstatus = HAL_SPI_Receive (&hspi1, &z1, 1, 10);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // chip selectOFF

		temp = ( temp2 << 8 ) | (( temp1 ) & 0xFF ); //combining the temp bits into one number

		if ((( y1 & 3 ) != 0 ) || (( z1 & 3 ) != 0 ) || (( x1 & 3 ) != 1 ))
			boolIsMisalligned = 1;

		if ((( y1 & 2 ) != 0 ) || (( z1 & 2 ) != 0 ) || (( x1 & 2 ) != 0 ))
			boolIsEmpty = 1;

		if ((boolIsMisalligned == 0) && (boolIsEmpty == 0)  && ( SPIstatus != HAL_TIMEOUT)) { //valid data

			x0 = ( x3 << 16 ) | ( x2 << 8 ) | ( x1 ); //rewriting the x,y,z bits into a single number
			y0 = ( y3 << 16 ) | ( y2 << 8 ) | ( y1 );
			z0 = ( z3 << 16 ) | ( z2 << 8 ) | ( z1 );

			//FIFO STUFF STARTS HERE
			if (index >= pFifoX->size-1){ //only take stuff out when its full
				fifo_read(pFifoX, &dataOutX, 1);
				fifo_read(pFifoY, &dataOutY, 1);
				fifo_read(pFifoZ, &dataOutZ, 1);

				if (boolWrote == 0)
					terminate++;
			}

			boolWrote = 1;

			if (terminate == pFifoX->size-2)
				break;

			fifo_write(pFifoX, &x0, 1);
			fifo_write(pFifoY, &y0, 1);
			fifo_write(pFifoZ, &z0, 1);

			averageX = fifo_average(pFifoX);
			averageY = fifo_average(pFifoY);
			averageZ = fifo_average(pFifoZ);


			if (index < pFifoX->size-1)
				index++;

			//FIFO STUFF ENDS HERE

			if (timeElapsed > delay*SECONDS/(pow(2,7))) { //(timeElapsed > delay*SECONDS/15)
                                            //magic number based on filter currently testing
				timeElapsed = 0;

// 				x = ADXL355_Acceleration_Data_Conversion (round(averageX)); //converting into acceleration data
//				y = ADXL355_Acceleration_Data_Conversion (round(averageY)); //x0,y0,z0 is average for FIFO stuff
//				z = ADXL355_Acceleration_Data_Conversion (round(averageZ));

				x = averageX;
				y = averageY;
				z = averageZ;

				xf = x; //copying this int into a float for easy conversions
				yf = y;
				zf = z;

				xf = ( xf * 1000000.0 / scale ); //scaling this value based off of the current range
				yf= ( yf * 1000000.0 / scale );
				zf = ( zf * 1000000.0 / scale );

				x = round(xf); //our values are in microGs, so we round to the nearest int
				y = round(yf); //this is consistent with the eval board's precision
				z = round(zf);

				temperature = (-1 * (temp - 1852.0) / 9.05 ) + 25 ; //converting temperature data into celsius
				floatToString(temperature, 4, temperatureString, 7); //converting this float into a printable string

				char stringAngleA[10];
				char stringAngleB[10];

				double angleA = atan2((double)y,(double)x);
				angleA *= 180.0;
				angleA = angleA / M_PI;

				double angleB = atan2((double)z,(double)x);
				angleB *= 180.0;
				angleB = angleB / M_PI;

				floatToString(angleA, 6, stringAngleA, 10);
				floatToString(angleB, 6, stringAngleB, 10);

				printf("\r\n,Angle A (deg):, %s,  Angle B (deg):, %s, "
						, stringAngleA, stringAngleB);
				printf("X (ug):, %08d,  Y (ug):, %08d,  Z (ug):, %08d,  Range (g):, %lu,  ACC Temp (C):, "
						"%s, Status:, %lu, ", x, y, z, range, temperatureString, ADXL355_SPI_Read(0x04));

				HAL_ADC_DeInit(&hadc);
				HAL_ADCEx_EnableVREFINTTempSensor(); //new
				SET_BIT(SYSCFG->CFGR3, SYSCFG_CFGR3_ENBUF_SENSOR_ADC);  //new
				HAL_ADC_Init(&hadc);
				HAL_ADC_Start(&hadc);

				printTempST();

				HAL_ADC_Stop(&hadc);
				HAL_ADCEx_DisableVREFINTTempSensor();
				HAL_ADC_DeInit(&hadc);
				HAL_ADC_Init(&hadc);
			}

			timeElapsed++;
			HAL_Delay(1);
			UARTstatus = HAL_UART_Receive(&huart2, (uint8_t *)in, 1, 1);

		} else {  //valid data end of if statement
				//boolWrote = 0;
				UARTstatus = HAL_UART_Receive(&huart2, (uint8_t *)in, 1, 1);
		}


	} while (UARTstatus != HAL_OK); //keep looping till they type


	ADXL355_Stop_Sensor(); //stop sensors
}

/*
 * @param : ui8address - unsigned 8 bit integer that represents the address we will read from
 *
 * This function is a callback for the accelerometer read
 */
uint32_t ADXL355_SPI_Read(uint8_t ui8address) {

	HAL_StatusTypeDef status;
	uint8_t recieveData;
	uint8_t txData;

	txData = (ui8address << 1) | 1 ;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //ON

	status = HAL_SPI_Transmit (&hspi1, &txData, 1, 100);

	status = HAL_SPI_Receive (&hspi1, &recieveData, 1, 100);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //OFF

	if (status == HAL_OK)
		return recieveData;
	else
		printf("\r\nError Reading: Invalid HAL_STATUS\r\n");

	return 255;
}
/*
 * @param : ui8address - unsigned 8 bit integer that represents the address we will write to
 * @param : ui8Data - unsigned 8 bit integer that represents the data we will write into the corresponding address
 * @param : enMode - regarding how many bytes of data you will write, i chose to not really use this and hardcoded a
 *                   1 for '1 byte' in every write I ever used
 *
 * This function is a callback for the accelerometer write
 */
void ADXL355_SPI_Write(uint8_t ui8address, uint8_t ui8Data, enWriteData enMode) {

	HAL_StatusTypeDef status;
	uint8_t address;

	address = ((ui8address << 1) & 0xFE);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //ON

	status = HAL_SPI_Transmit (&hspi1, &address, 1, 100);
	status = HAL_SPI_Transmit (&hspi1, &ui8Data, 1, 100);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //OFF

	if (status != HAL_OK)
		printf("\r\nError writing: Invalid HAL STATUS\r\n");
}

/*
 * @param : *f - FIFO pointer so that we can access its contents
 * @param : *buf - pointer to buffer string the FIFO will be utilizing
 * @param : size - size of the buffer string
 *
 * This function initializes the FIFO structure with the given buffer and size
 */
void fifo_init(fifo_t * f, uint32_t * buf, int size) {
	f -> head = 0;
	f -> tail = 0;
	f -> size = size;
	f -> buf = buf;
}
/*
 * @param : *f - FIFO pointer so that we can access its contents
 * @param : *buf - pointer to data string that we will be reading from
 * @param : nbytes - number of bytes we will be reading
 *
 * This function reads nbytes bytes from the FIFO, and the number of bytes read is returned
 * When something is read, something is popped off the FIFO, aka the lagging tail increases
 */
int fifo_read(fifo_t * f, void * buf, int nbytes) {
	int i;
	uint32_t * p;
	p = buf;

	for( i=0 ; i < nbytes ; i++ ) {
		if( f -> tail != f -> head ){ //see if any data is available
			*p++ = f -> buf[f -> tail];  //grab a byte from the buffer
			f -> tail++;  //increment the tail
			if( f -> tail == f -> size )  //check for wrap-around
				f -> tail = 0;
          } else //if data is not available
        	  return i; //number of bytes read
     }
     return nbytes;
}
/*
 * @param : *f - FIFO pointer so that we can access its contents
 * @param : *buf - pointer to data string that we will write into
 * @param : nbytes - number of bytes we will be writing
 *
 * This function writes up to nbytes bytes to the FIFO.
 * The number of bytes written is returned.
 * When something is written to, data is pushed onto the FIFO, aka the leading head increases
 * If the head runs in to the tail, data is overwritten
 */
int fifo_write(fifo_t * f, const void * buf, int nbytes) {
     int i;
     const uint32_t * p;
     p = buf;
     for( i=0 ; i < nbytes ; i++ ){
           //first check to see if there is space in the buffer
           if( (f -> head + 1 == f -> tail) || ((f -> head + 1 == f -> size) && (f -> tail == 0) )) {
        	   //no more room , overwrite
        	   f -> buf[f -> head] = *p++;
        	   f -> head++;  //increment the head
        	   f -> tail++; //increment tail (for overwrite)
        	   if ( f -> head == f -> size )  //check for wrap-around
        		   f -> head = 0;
        	   if (f -> tail == f -> size)
        		   f -> tail = 0;
           } else {
        	   f -> buf[f -> head] = *p++;
        	   f -> head++;  //increment the head
        	   if ( f -> head == f -> size )  //check for wrap-around
        		   f -> head = 0;
           }
     }
     return nbytes;
}
/*
 * @param : *f - FIFO pointer so that we can access its contents
 *
 * This function takes the average of all values contained in the FIFO buffer
 * In our convention, the number of items in a FIFO buffer of size n = n-1
 * This function returns the average as a float
 */
float fifo_average(fifo_t * f) { //tail is in front
	int64_t total = 0;
	float counter = 0.0;
	//int32_t print;

	//printf("\r\nNumbers:");
	int tempTail = f->tail;

	//z = ADXL355_Acceleration_Data_Conversion ();

	while (1) {

		if( tempTail != f -> head ){ //see if any data is available
			//print = f -> buf[tempTail];
			//printf(" %ld", print);
			total += ADXL355_Acceleration_Data_Conversion (f -> buf[tempTail]);  //adding to total NEW THING IS THE CONVERSION
			counter++;
			tempTail++;  //increment the tail
			if( tempTail == f -> size )  //check for wrap-around
				tempTail = 0;
		} else //if data is not available
			break; //number of bytes read
		}
	//printf("\r\n");

	float fTotal = total + 0.0;

	if ( counter == 0 )
		return 0;
	else
		return (fTotal/counter);
}
/*
 * This function was written with the sole purpose of debugging the software FIFO
 * It let me test it so that I could eventually get it to work
 * It was originally in main, with everything else commented out
 */
void fifoDebugger(void) {

	float average;
	char averageString[6];
	int terminate = 0;
	int boolWrote = 0;
	uint32_t buffer[BUFFER_SIZE];
	uint32_t dataOut; //sentinel value

	uint32_t data[15] = {5, 13, 4, 22, 46, 2, 19, 12, 56, 71, 33, 22, 15, 64, 24};
	//uint32_t data[DATA_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //useful for debugging

	fifo_t myFifo; //fifo declaration

	uint32_t *pBuffer = &buffer[0]; //pointer declarations
	fifo_t *pFifo = &myFifo;
	uint32_t *pData = &data[0];

	fifo_init(pFifo, pBuffer, BUFFER_SIZE); //initialization

    printf("\r\nStarting. . .\r\n");

    for (int i = 0 ; i < 100 ; i++) {

    	if (i >= pFifo->size-1){
    		fifo_read(pFifo, &dataOut, 1);
		    if (boolWrote == 0)
		    	terminate++;
    	}

    	if (terminate == pFifo->size-2)
    		break;
    	boolWrote = 0;

    	if (i < 15) {
    		fifo_write(pFifo, pData++, 1);
    		boolWrote = 1;
    	}

    	average = fifo_average(pFifo);
    	floatToString(average, 4, averageString, 6);

    	printf("Average: %s \r\n\n", averageString);
    	//printf("Read: %lu  Write: %lu\r\n", dataOut, *pData++); //DO NOT INCREMENT POINTER IN MORE THAN ONE PLACE !!!
    }

    printf("Done!\r\n");
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

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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "max6650.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Flash allocated strings
static const char digitString[] = "0123456789";
static const char setFanSpeed[] = "set_fan_speed ";
static const char getFanSpeed[] = "get_fan_speed";
static const char selfErase[] = "self_erase";
static const char wrongValue[] = "Error: wrong value";
static const char unknownCommand[] = "Error: unknown command";
static const char libraryError[] = "Error: library error";
static const char areYouSure[] = "Are you sure? (y/n)";
static const char yes1[] = "y";
static const char yes2[] = "y\r";
static const char noFunctional[] = "No functional\r\n";

// HAL periphery timeouts
static const int i2cTimeout = 100;
static const int uartTimeout = 100;

// Terminal buffer size
static const int textBufferSize = 100;

// Command handlers and table
void setFanSpeedHandler(const char *);
void getFanSpeedHandler(const char *);
void selfEraseHandler  (const char *);
CommandHandler commandHandler[] = {{setFanSpeed, setFanSpeedHandler},
                                   {getFanSpeed, getFanSpeedHandler},
                                   {selfErase,   selfEraseHandler  }};


// Erases Flash and answers "No functional" to every terminal command
// Allocated in RAM
__RAM_FUNC void eraseFlash(){

  // Move noFunctional string to RAM
  char * txBuffer = strdup(noFunctional);

  // Unlock FLASH
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;

  // Erase FLASH
  while((FLASH->CR & FLASH_SR_BSY) != 0);
  FLASH->CR |= FLASH_CR_MER;
  FLASH->CR |= FLASH_CR_STRT;
  while((FLASH->CR & FLASH_SR_BSY) != 0);

  // Lock FLASH
  FLASH->CR |= FLASH_CR_LOCK;

  // Answer "No functional" to every command
  while(1){

    // Receive characters from terminal untl EOL
    while((USART1->SR & UART_FLAG_RXNE) == 0);
    if((char)(USART1->DR & 0x00FFU) == '\n'){

      // Send "No functional" to terminal
      char * ptr = txBuffer;
      while(*ptr != '\0'){
        while((USART1->SR & UART_FLAG_TXE) == 0);
        USART1->DR = (uint8_t) *(ptr++);
      }
    }
  }
}

// Sends specified string end EOL characters to terminal
void sendStringToUart(const char * string){
  static const char nextString[] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *) string,     strlen(string),     uartTimeout);
  HAL_UART_Transmit(&huart1, (uint8_t *) nextString, strlen(nextString), uartTimeout);
}

// Receives string from terminal to buffer, max. bufferSize characters
// If received string is longer than bufferSize, first bufferSize characters will be lost
void getStringFromUart(char * buffer, size_t bufferSize){
  const char * bufferEnd = buffer + bufferSize;
  char * bufferIterator = buffer;
  do{
    if(bufferIterator == bufferEnd){
      bufferIterator = buffer;
    }
    while(HAL_UART_Receive(&huart1, (uint8_t *) bufferIterator, 1, uartTimeout) != HAL_OK);
  }while(*(bufferIterator++) != '\n');
  *(--bufferIterator) = '\0';
}

// Switshes I2C speed
// fast_speed == false => 100kHz
// fast_speed == false => 400kHz
// Returns true if switched successfully
bool setup_i2c(bool fast_speed){
  if(HAL_I2C_DeInit(&hi2c1) != HAL_OK){
    return false;
  }
  hi2c1.Init.ClockSpeed = fast_speed ? 400000 : 100000;
  if(HAL_I2C_Init(&hi2c1) != HAL_OK){
    return false;
  }
  return true;
}

// Writes (size) bytes from (buf) to the I2C device with (slave) bus address starting from (reg) internal address
// Returns true if data was wrote successfully
bool write_i2c(uint16_t slave, uint16_t reg, uint8_t* buf, uint16_t size){
  return HAL_I2C_Mem_Write(&hi2c1, slave, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, size, i2cTimeout) == HAL_OK;
}

// Reads (size) bytes to (buf) from the I2C device with (slave) bus address starting from (reg) internal address
// Returns true if data was read successfully
bool read_i2c(uint16_t slave, uint16_t reg, uint8_t* buf, uint16_t size){
  return HAL_I2C_Mem_Read(&hi2c1, slave, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, size, i2cTimeout) == HAL_OK;
}

// Sets specified speed of the fan (in %)
void setFanSpeedHandler(const char * parameter){

  // Searching for the digital value
  if(strpbrk(parameter, digitString) == parameter){
    sendStringToUart(wrongValue);
    return;
  }

  // Check if 1-3 digits detected as parameter in the command string
  size_t valueLength = strspn(parameter, digitString);
  if(valueLength > 3){
    sendStringToUart(wrongValue);
    return;
  }

  // Convert string value to uint16_t
  uint16_t value = 0;
  const char * ptr = parameter;
  while(valueLength-- > 0){
    value *= 10;
    value += *(ptr++) - '0';
  }

  // Check if value is OK for library function
  if(value > MAX6650_MAX_FAN_VALUE){
    sendStringToUart(wrongValue);
    return;
  }

  // Send value to the library function
  if(!MAX6650_setValue(value)){
    sendStringToUart(libraryError);
  }
}

// Reads current speed of the fan (in % and RPM) from MAX6650 library and sends them to terminal
void getFanSpeedHandler(const char * parameter){
  UNUSED(parameter);

  // Read current fan speed from MAX6650 library
  MAX6650_FanParameters fanParameters = MAX6650_getFanParameters();

  // Send received value to the terminal
  if(fanParameters.isValid){
    char textBuffer[20];
    sprintf(textBuffer, "%u%% (%u RPM)", fanParameters.value, fanParameters.rpm);
    sendStringToUart(textBuffer);
  }else{
    sendStringToUart(libraryError);
  }
}

void selfEraseHandler(const char * parameter){
  UNUSED(parameter);

  // Check if FLASH realy should be erased
  sendStringToUart(areYouSure);
  char textBuffer[5];
  getStringFromUart(textBuffer, sizeof(textBuffer) / sizeof(textBuffer[0]));
  if((strstr(textBuffer, yes1) == textBuffer) || (strstr(textBuffer, yes2) == textBuffer)){
    eraseFlash();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // MAX6650 initialization: I2C address pin is unconnected, I2C speed == 400kHz
  //                         fan voltage == 12V, speed: 1% == 500RPM, 100% == 10500RPM
  MAX6650_init(MAX6650_ADDR_HIGH_Z,
               MAX6650_I2C_SPEED_400K,
               MAX6650_FAN_VOLTAGE_12V,
               500,
               10500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Terminal buffer
  char textBuffer[textBufferSize];

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Read string from terminal
    getStringFromUart(textBuffer, textBufferSize);

    // Try to find a command in the list and execute its handler
    size_t i;
    for(i = 0; i < (sizeof(commandHandler) / sizeof(commandHandler[0])); ++i){
      if(strstr(textBuffer, commandHandler[i].command) == textBuffer){
        commandHandler[i].handler(textBuffer + strlen(commandHandler[i].command));
        break;
      }
    }

    // If command is not found send the notification to the terminal
    if(i == (sizeof(commandHandler) / sizeof(commandHandler[0]))){
      sendStringToUart(unknownCommand);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "stdarg.h"
#include "lfs_util.h"
#include "lfs.h"
#include "MT25Q.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define CDC_USB_DEBUG
#define UART_DEBUG
//internal
typedef struct{
	uint32_t secCount;
	uint32_t bootCount;
}app_count_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void myprintf(const char *fmt, ...);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t DEBUG_DATA_RX_FLAG = 0;
// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;
int erase_done = 0;
int mount_done = 0;
int format_done = 0;

//internal
lfs_t Lfs;
struct lfs_config LfsConfig;

app_count_t Counter = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int block_device_read(const struct lfs_config *c, lfs_block_t block,
//		lfs_off_t off, void *buffer, lfs_size_t size) {
//	Bulk_Read_4B(&hspi2, (block * c->block_size + off), (uint8_t*) buffer,
//			size);
//	return 0;
//}
//
//int block_device_prog(const struct lfs_config *c, lfs_block_t block,
//		lfs_off_t off, const void *buffer, lfs_size_t size) {
//	Page_Write_4B(&hspi2, (block * c->block_size + off), (uint8_t *) buffer,
//			size);
//	return 0;
//}
//
//int block_device_erase(const struct lfs_config *c, lfs_block_t block){
//	Sector_Erase_4B(&hspi2, block, 4);
//	return 0;
//}
//
//
//int block_device_sync(const struct lfs_config *c){
//	return 0;
//}
//
//const struct lfs_config cfg = {
//    // block device operations
//    .read  = block_device_read,
//    .prog  = block_device_prog,
//    .erase = block_device_erase,
//	.sync  = block_device_sync,
//
//    // block device configuration
//    .read_size = 1,
//    .prog_size = 1,
//    .block_size = 65536,
//    .block_count = 2048,
//    .cache_size = 16,
//    .lookahead_size = 16,
//    .block_cycles = 500,
//};

//internal
int _flash_read(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size){

	uint32_t StartAddress, i;
	uint32_t *Bff;

	StartAddress = 0x0800D800 + (block*c->block_size) + off;

	Bff = (uint32_t*)buffer;
	for (i=0 ; i<(size/4) ; i++){
		*Bff = *(__IO uint32_t*)StartAddress;
		StartAddress += 4;
		Bff++;
	}

	return LFS_ERR_OK;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int _flash_prog(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size){
	uint32_t StartAddress, i;
	uint32_t *Bff;

	StartAddress = 0x0800D800 + (block*c->block_size) + off;
	Bff = (uint32_t*)buffer;

	HAL_FLASH_Unlock();
	for (i=0 ; i<(size/4) ; i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartAddress, (uint64_t)(*Bff));
		StartAddress += 4;
		Bff++;
	}
	HAL_FLASH_Lock();

	return LFS_ERR_OK;
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int _flash_erase(const struct lfs_config *c, lfs_block_t block){
	uint32_t StartAddress;
	uint32_t PageErr;
	FLASH_EraseInitTypeDef EraseInitStruct;

	StartAddress = 0x0800D800 + (block*c->block_size);
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartAddress;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.Banks = FLASH_BANK_1;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageErr);
	HAL_FLASH_Lock();

	return LFS_ERR_OK;
}

// Sync the state of the underlying block device. Negative error codes
// are propagated to the user.
int _flash_sync(const struct lfs_config *c){
	return LFS_ERR_OK;
}

void __init_storage(){
	int32_t error;

	LfsConfig.read_size = 64;
	LfsConfig.prog_size = 64;
	LfsConfig.block_size = 1024;
	LfsConfig.block_count = 10;
	LfsConfig.cache_size = 256;
	LfsConfig.lookahead_size = 8;
	LfsConfig.block_cycles = 1000;

	LfsConfig.read = _flash_read;
	LfsConfig.prog = _flash_prog;
	LfsConfig.erase = _flash_erase;
	LfsConfig.sync = _flash_sync;

	error = lfs_mount(&Lfs, &LfsConfig);
	if (error != LFS_ERR_OK){
		lfs_format(&Lfs, &LfsConfig);
		error = lfs_mount(&Lfs, &LfsConfig);
		if (error != LFS_ERR_OK){
			Error_Handler();
		}
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
	uint32_t HalTickAux;
	lfs_file_t File;
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  int add = 0;
//  for(int i=0;i<300;i++){
//	  Sector_Erase_4B(&hspi2, add, 64);
//	  add+=65536;
//  }
  erase_done = 1;
//  myprintf("Starting LittleFS application...\n");
  //internal
  HAL_Delay(100);
   __init_storage();

   lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDONLY | LFS_O_CREAT);
   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
   lfs_file_close(&Lfs, &File);

   Counter.bootCount += 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		int err = lfs_mount(&lfs, &cfg);
//
//		// reformat if we can't mount the filesystem
//		// this should only happen on the first boot
//		if (err) {
//			format_done = lfs_format(&lfs, &cfg);
//			mount_done = lfs_mount(&lfs, &cfg);
//		}
//		// read current count
//		uint32_t boot_count = 0;
//		int ret = lfs_file_open(&lfs, &file, "boot_count.txt", LFS_O_CREAT);
//		if(ret < 0){
//			myprintf("Unable to open file...\n Trying again...\n");
//			ret = lfs_file_open(&lfs, &file, "boot_count.txt", LFS_O_RDONLY | LFS_O_CREAT);
//		}
//		int ret1 = lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
//
//		// update boot count
//		boot_count += 1;
//		lfs_file_rewind(&lfs, &file);
//		lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
//
//		// remember the storage is not updated until the file is closed successfully
//		lfs_file_close(&lfs, &file);
//
//		// release any resources we were using
//		lfs_unmount(&lfs);
//
//		// print the boot count
//		myprintf("boot_count: %d\n", (uint8_t)boot_count);
		//internal

		  HalTickAux = HAL_GetTick();

		  lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDWR | LFS_O_CREAT);
		  lfs_file_write(&Lfs, &File, &Counter, sizeof(app_count_t));
		  lfs_file_close(&Lfs, &File);

		  while ((HAL_GetTick() - HalTickAux) < 1000);

		  Counter.secCount += 1;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FM_CS_GPIO_Port, FM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FM_CS_Pin */
  GPIO_InitStruct.Pin = FM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FM_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void myprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[100];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    va_end(args);
}

int bufferSize(char *buffer) {
    int i = 0;
    while (*buffer++ != '\0')
        i++;
    return i;
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
	while (1) {
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

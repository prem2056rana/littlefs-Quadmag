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
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>

#include "lfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct{
	uint32_t secCount;
	uint32_t bootCount;
}app_count_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
lfs_t Lfs;
struct lfs_config LfsConfig;

app_count_t Counter = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Read a region in a block. Negative error codes are propagated
// to the user.
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
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  __init_storage();

  lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDONLY | LFS_O_CREAT);
  lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
  lfs_file_close(&Lfs, &File);

  Counter.bootCount += 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HalTickAux = HAL_GetTick();

	  lfs_file_open(&Lfs, &File, "count.bin", LFS_O_RDWR | LFS_O_CREAT);
	  lfs_file_write(&Lfs, &File, &Counter, sizeof(app_count_t));
	  lfs_file_close(&Lfs, &File);

	  while ((HAL_GetTick() - HalTickAux) < 1000);

	  Counter.secCount += 1;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

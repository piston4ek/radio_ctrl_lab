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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEADER 0x55
// Status
#define WITHOUT_OP    0x00
#define OK            0x01
#define INVALID_HDR   0x02
#define INVALID_CRC   0x03
#define UNDEFINED_ERR 0x04

#define CH_MHZ 2500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {0, 1, 2};
//uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = {0, 1, 2};
uint16_t adc_val[1];
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
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//HAL_ADCEx_Calibration_Start();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 1);
	
	nrf24l01p_tx_init(CH_MHZ, _1Mbps);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_ADC_Start_DMA(
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//int tx_mode = 1;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//UNUSED(GPIO_Pin);
	
	if(GPIO_Pin == Push_Button_Pin)
	{
		//nrf24l01p_ptx_mode();
		//tx_mode = 1;
		char msg [150];
		sprintf(msg, "x = %0d; y = %0d\n\r", adc_val[0],adc_val[0] /*adc_val[1]*/);
		CDC_Transmit_FS((uint8_t*)msg , (uint16_t)strlen(msg));
		
		tx_data[0] = HEADER;
		//tx_data[1] = adc_val[0] & 0xFF00; // MSB
		//tx_data[2] = adc_val[0] & 0x00FF; // LSB
		//tx_data[3] = adc_val[1] & 0xFF00; // MSB
		//tx_data[4] = adc_val[1] & 0x00FF; // LSB
		tx_data[5] = 0x00;
		for(int i = 0; i < 5; ++i)
		{
			tx_data[5]+= tx_data[i];
		}

		nrf24l01p_tx_transmit(tx_data);
		
		//nrf24l01p_prx_mode();
		//tx_mode = 0;
		
		HAL_GPIO_TogglePin(USB_LED_GPIO_Port, USB_LED_Pin);
	}
	
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		//if(tx_mode)
		//{
			nrf24l01p_tx_irq(); // clear interrupt flag
		//}
		/*else
		{
			// RCV Respond
			//nrf24l01p_tx_irq();
			char msg[200];
			//HAL_Delay(100);
			nrf24l01p_rx_receive(rx_data);
			
			if(rx_data[0] == HEADER)
			{
				if((rx_data[0] + rx_data[1]) == rx_data[2])
				{
					sprintf(msg, "\n[MASTER] : RESPOND FROM SLAVE:\n\r0: 0x%0x\n\r1: 0x%0x\n\r2: 0x%0x\n\rCommunication established\n\r", 
            rx_data[0], rx_data[1], rx_data[2]);
				}
				else
				{
					sprintf(msg, "\n[MASTER] : RESPOND FROM SLAVE:\n\r0: 0x%0x\n\r1: 0x%0x\n\r2: 0x%0x\n\rCRC Mismatched\n\r", 
            rx_data[0], rx_data[1], rx_data[2]);
				}
			}
			else
			{
				//sprintf(additional_txt, "Invalid Header\n\r");
				sprintf(msg, "\n[MASTER] : RESPOND FROM SLAVE:\n\r0: 0x%0x\n\r1: 0x%0x\n\r2: 0x%0x\n\rInvalid Header\n\r", 
            rx_data[0], rx_data[1], rx_data[2]);
			}
		
			CDC_Transmit_FS((uint8_t*)msg, (uint16_t)strlen(msg));
			
			nrf24l01p_ptx_mode();
			tx_mode = 1;
		}*/
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

#include "current_sense/hardware_api.h"
#if defined(ARDUINO_PHOQUE1)

#include "communication/SimpleFOCDebug.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_hal_adc.h"
#include "phoque1_hal.h"

// From STM32 cube IDE
/**
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

#ifndef READ_BEMF
#define READ_BEMF 0
#endif

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	__HAL_RCC_ADC12_CLK_ENABLE();
}

/** 
* Enable DMA controller clock
*/
void  MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	// Enable external clock for ADC
	RCC_PeriphCLKInitTypeDef PeriphClkInit;
	PeriphClkInit.PeriphClockSelection   = RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Adc12ClockSelection    = RCC_ADC12CLKSOURCE_PLL;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}


/**
* @brief ADC1 Initialization Function
* @param None
* @retval None
*/
void  MX_ADC1_Init(ADC_HandleTypeDef* hadc1)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config 
	*/
	hadc1->Instance = ADC1;
	hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
	hadc1->Init.Resolution = ADC_RESOLUTION_12B;
	hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1->Init.GainCompensation = 0;
	hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1->Init.LowPowerAutoWait = DISABLE;
	hadc1->Init.ContinuousConvMode = DISABLE;

	hadc1->Init.NbrOfConversion = 3 + READ_BEMF * 2;

	hadc1->Init.DiscontinuousConvMode = DISABLE;
	hadc1->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1->Init.DMAContinuousRequests = ENABLE;
	hadc1->Init.Overrun = ADC_OVR_DATA_PRESERVED;

	if (HAL_ADC_Init(hadc1) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_Init failed!");
	}

	/** Configure the ADC multi-mode 
	*/
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADCEx_MultiModeConfigChannel 1 failed!");
	}
	/** Configure Regular Channel (Opamp 1 / phase 3 current)
	*/
	sConfig.Channel = ADC_CHANNEL_13;  // OP1_OUT is ADC1_IN13 for internal channel
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	#if READ_BEMF
	/* Configure Regular Channel (PA2 / BEMF1 / Phase 3)
	*/
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	/* Configure Regular Channel (PB1 / BEMF3 / Phase 1)
	*/
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	#endif

	//******************************************************************
	// Aux analog readings
	/* Configure Regular Channel (PF0, motor temperature)
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	#if READ_BEMF
	sConfig.Rank = ADC_REGULAR_RANK_4;
	#else
	sConfig.Rank = ADC_REGULAR_RANK_2;
	#endif
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	/** Configure Regular Channel (PC1, Potentiometer)
	*/
	sConfig.Channel = ADC_CHANNEL_7;
	#if READ_BEMF
	sConfig.Rank = ADC_REGULAR_RANK_5;
	#else
	sConfig.Rank = ADC_REGULAR_RANK_3;
	#endif
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
}

/**
* @brief ADC2 Initialization Function
* @param None
* @retval None
*/
void MX_ADC2_Init(ADC_HandleTypeDef* hadc2)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config 
	*/
	hadc2->Instance = ADC2;
	hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
	hadc2->Init.Resolution = ADC_RESOLUTION_12B;
	hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2->Init.GainCompensation = 0;
	hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2->Init.LowPowerAutoWait = DISABLE;
	hadc2->Init.ContinuousConvMode = DISABLE;

	hadc2->Init.NbrOfConversion = 4 + READ_BEMF;

	hadc2->Init.DiscontinuousConvMode = DISABLE;
	hadc2->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2->Init.DMAContinuousRequests = ENABLE;
	hadc2->Init.Overrun = ADC_OVR_DATA_PRESERVED;

	if (HAL_ADC_Init(hadc2) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_Init failed!");
	}

	/** Configure Regular Channel (Opamp 2 / phase 2 current)
	*/
	sConfig.Channel = ADC_CHANNEL_16;  // OP2_OUT is ADC2_IN16 for internal channel
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	/** Configure Regular Channel (Opamp 3 / phase 1 current)
	*/
	sConfig.Channel = ADC_CHANNEL_18;     // OP3_OUT is ADC2_IN18 for internal channel
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	#if READ_BEMF
	/** Configure Regular Channel (PC5 / BEMF2 / Phase 2)
	*/
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	#endif

	/** Configure Regular Channel (PF1 / Mosfet temperature)
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	#if READ_BEMF
	sConfig.Rank = ADC_REGULAR_RANK_4;
	#else
	sConfig.Rank = ADC_REGULAR_RANK_3;
	#endif
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	/** Configure Regular Channel (PC0 / Bus voltage monitor)
	*/
	sConfig.Channel = ADC_CHANNEL_6;
	#if READ_BEMF
	sConfig.Rank = ADC_REGULAR_RANK_5;
	#else
	sConfig.Rank = ADC_REGULAR_RANK_4;
	#endif
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

}

/**
* @brief OPAMP MSP Initialization
* This function configures the hardware resources used in this example
* @param hopamp-> OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hopamp->Instance==OPAMP1)
	{
	/* USER CODE BEGIN OPAMP1_MspInit 0 */

	/* USER CODE END OPAMP1_MspInit 0 */

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**OPAMP1 GPIO Configuration    
		PA1     ------> OPAMP1_VINP
		PA2     ------> OPAMP1_VOUT
		PA3     ------> OPAMP1_VINM 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN OPAMP1_MspInit 1 */

	/* USER CODE END OPAMP1_MspInit 1 */
	}
	else if(hopamp->Instance==OPAMP2)
	{
	/* USER CODE BEGIN OPAMP2_MspInit 0 */

	/* USER CODE END OPAMP2_MspInit 0 */

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**OPAMP2 GPIO Configuration    
		PA5     ------> OPAMP2_VINM
		PA6     ------> OPAMP2_VOUT
		PA7     ------> OPAMP2_VINP 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN OPAMP2_MspInit 1 */

	/* USER CODE END OPAMP2_MspInit 1 */
	}
	else if(hopamp->Instance==OPAMP3)
	{
	/* USER CODE BEGIN OPAMP3_MspInit 0 */

	/* USER CODE END OPAMP3_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**OPAMP3 GPIO Configuration    
		PB0     ------> OPAMP3_VINP
		PB1     ------> OPAMP3_VOUT
		PB2     ------> OPAMP3_VINM 
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN OPAMP3_MspInit 1 */

	/* USER CODE END OPAMP3_MspInit 1 */
	}

}

/**
* @brief OPAMP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hopamp-> OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp)
{
	if(hopamp->Instance==OPAMP1)
	{
	/* USER CODE BEGIN OPAMP1_MspDeInit 0 */

	/* USER CODE END OPAMP1_MspDeInit 0 */

		/**OPAMP1 GPIO Configuration    
		PA1     ------> OPAMP1_VINP
		PA2     ------> OPAMP1_VOUT
		PA3     ------> OPAMP1_VINM 
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_3);

	/* USER CODE BEGIN OPAMP1_MspDeInit 1 */

	/* USER CODE END OPAMP1_MspDeInit 1 */
	}
	else if(hopamp->Instance==OPAMP2)
	{
	/* USER CODE BEGIN OPAMP2_MspDeInit 0 */

	/* USER CODE END OPAMP2_MspDeInit 0 */

		/**OPAMP2 GPIO Configuration    
		PA5     ------> OPAMP2_VINM
		PA6     ------> OPAMP2_VOUT
		PA7     ------> OPAMP2_VINP 
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_7);

	/* USER CODE BEGIN OPAMP2_MspDeInit 1 */

	/* USER CODE END OPAMP2_MspDeInit 1 */
	}
	else if(hopamp->Instance==OPAMP3)
	{
	/* USER CODE BEGIN OPAMP3_MspDeInit 0 */

	/* USER CODE END OPAMP3_MspDeInit 0 */

		/**OPAMP3 GPIO Configuration    
		PB0     ------> OPAMP3_VINP
		PB1     ------> OPAMP3_VOUT
		PB2     ------> OPAMP3_VINM 
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_2);

	/* USER CODE BEGIN OPAMP3_MspDeInit 1 */

	/* USER CODE END OPAMP3_MspDeInit 1 */
	}

}

#endif 
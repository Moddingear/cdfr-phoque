#include "current_sense/hardware_api.h"
#if defined(ARDUINO_PHOQUE2)

#include "communication/SimpleFOCDebug.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_hal_adc.h"
#include "phoque2_hal.h"

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
	sConfig.Channel = ADC_CHANNEL_13;
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
	/* Configure Regular Channel (PB0 / BEMFV / Phase V)
	*/
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	/* Configure Regular Channel (PB1 / BEMFW / Phase W)
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
	/* Configure Regular Channel (PB12, mosfet temperature)
	*/
	sConfig.Channel = ADC_CHANNEL_11;
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

	/** Configure Regular Channel (PA3 / Bus voltage monitor)
	*/
	sConfig.Channel = ADC_CHANNEL_4;
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

	hadc2->Init.NbrOfConversion = 3 + READ_BEMF;

	hadc2->Init.DiscontinuousConvMode = DISABLE;
	hadc2->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
	hadc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2->Init.DMAContinuousRequests = ENABLE;
	hadc2->Init.Overrun = ADC_OVR_DATA_PRESERVED;

	if (HAL_ADC_Init(hadc2) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_Init failed!");
	}

	/** Configure Regular Channel (PA0 / phase U current)
	*/
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	/** Configure Regular Channel (PC4 / phase V current)
	*/
	sConfig.Channel = ADC_CHANNEL_5;
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
	/** Configure Regular Channel (PA0 / BEMFU / Phase U)
	*/
	sConfig.Channel = ADC_CHANNEL_1;
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

	/** Configure Regular Channel (PA4 / Potentiometer)
	*/
	sConfig.Channel = ADC_CHANNEL_17;
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
}

#endif 
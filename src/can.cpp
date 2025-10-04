#include <can.hpp>

#include <Arduino.h>
#include <stm32g4xx_hal.h>
#include <hal.hpp>

#if HAL_CAN
FDCAN_HandleTypeDef hfdcan1 = {};


void ArduinoPinToHAL(uint32_t APin, uint32_t* HALPin, GPIO_TypeDef** HALPort)
{
	uint32_t Pin = digitalPinToPinName(APin);
	*HALPort = get_GPIO_Port(STM_PORT(Pin));
	*HALPin  = STM_LL_GPIO_PIN(Pin);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == NULL || hfdcan->Instance != FDCAN1)
	{
		return;
	}

	RCC_PeriphCLKInitTypeDef periphClkInit = { };
	
	HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);
	
	// Initializes the peripherals clocks
	periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
	periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	
	// Peripheral clock enable
	__HAL_RCC_FDCAN_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// FDCAN1 GPIO Configuration
	// PA11 ------> FDCAN1_RX
	uint32_t Pin;
	GPIO_TypeDef*    Port;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	ArduinoPinToHAL(A_CAN_RX, &Pin, &Port);
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(Port, &GPIO_InitStruct);

	// PB9/PA12 ------> FDCAN1_TX 
	ArduinoPinToHAL(A_CAN_TX, &Pin, &Port);
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(Port, &GPIO_InitStruct);


	// FDCAN1 interrupt Init
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

void CAN::init(uint16_t identifier, uint64_t frequency)
{
	uint32_t base_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
	uint64_t clock_cycles_per_bit = base_freq/frequency;
	if (base_freq % frequency != 0)
	{
		Error_Handler();
	}
	
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = ENABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;

	hfdcan1.Init.NominalPrescaler = clock_cycles_per_bit/17;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 14;
	hfdcan1.Init.NominalTimeSeg2 = 2;

	if (clock_cycles_per_bit%17 != 0)
	{
		Error_Handler();
	}
	

	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 4;
	hfdcan1.Init.DataTimeSeg1 = 5;
	hfdcan1.Init.DataTimeSeg2 = 4;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = identifier;
	sFilterConfig.FilterID2 = 0x7ff;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) 
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
		Error_Handler();
	}
}
#endif
#pragma once

#if defined(ARDUINO_PHOQUE2)

#include <stm32g4xx_hal_adc.h>
#include <stm32g4xx_hal_opamp.h>

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(ADC_HandleTypeDef* hadc1);
void MX_ADC2_Init(ADC_HandleTypeDef* hadc2);

#endif
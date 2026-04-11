#include "current_sense/hardware_api.h"

#if defined(ARDUINO_PHOQUE2) 

#include "phoque2_hal.h"
#include "Arduino.h"
#include "current_sense/hardware_specific/stm32/stm32_mcu.h"
#include "communication/SimpleFOCDebug.h"
#include "current_sense/hardware_specific/stm32/stm32_adc_utils.h"
#include "stm32g4xx_hal_conf.h"

#ifndef READ_BEMF
#define READ_BEMF 0
#endif

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4096.0f
#define ADC_BUF_LEN_1 (3 + READ_BEMF*2)
#define ADC_BUF_LEN_2 (3 + READ_BEMF)

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;

volatile uint16_t adcBuffer1[ADC_BUF_LEN_1] = {0}; // Buffer for store the results of the ADC conversion
volatile uint16_t adcBuffer2[ADC_BUF_LEN_2] = {0}; // Buffer for store the results of the ADC conversion



// structure containing the configuration of the adc interrupt
Stm32AdcInterruptConfig adc_interrupt_config[5] = {
	{0, 0, 0}, // ADC1
	{0, 0, 0}, // ADC2
	{0, 0, 0}, // ADC3
	{0, 0, 0}, // ADC4
	{0, 0, 0}  // ADC5
};

// function reading an ADC value and returning the read voltage
// As DMA is being used just return the DMA result
float _readADCVoltageLowSide(const int pin, const void* cs_params){
	uint32_t raw_adc;
	
	switch (pin)
	{
	case A_CURRU:
	case -1:
		raw_adc = adcBuffer1[0];
		break;
	case A_CURRV:
	case -2:
		raw_adc = adcBuffer2[0];
		break;
	case A_CURRW:
	case -3:
		raw_adc = adcBuffer2[1];
		break;
	#if READ_BEMF
	case A_BEMFU:
		raw_adc = adcBuffer2[2];
		break;
	case A_BEMFV:
		raw_adc = adcBuffer1[1];
		break;
	case A_BEMFW:
		raw_adc = adcBuffer1[2];
		break;
	#endif
	case A_POTENTIOMETER:
		raw_adc = adcBuffer2[2+READ_BEMF];
		break;
	case A_TEMPERATURE:
		raw_adc = adcBuffer1[1+READ_BEMF*2];
		break;
	case A_VBUS:
		raw_adc = adcBuffer1[2+READ_BEMF*2];
		break;
	default:
		raw_adc = 0;
		break;
	}
	return raw_adc * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
}

void MX_DMA1_Init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef* channel, uint32_t request) {
	hdma_adc->Instance = channel;
	hdma_adc->Init.Request = request;
	hdma_adc->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc->Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc->Init.Mode = DMA_CIRCULAR;
	hdma_adc->Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_DeInit(hdma_adc);
	if (HAL_DMA_Init(hdma_adc) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_DMA_Init failed!");
	}
	__HAL_LINKDMA(hadc, DMA_Handle, *hdma_adc);
}

void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
	_UNUSED(driver_params);
	_UNUSED(pinA);
	_UNUSED(pinB);
	_UNUSED(pinC);

	SIMPLEFOC_DEBUG("Phoque2 does not implement inline current sense. Use low-side current sense instead.");
	return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}


void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
	_UNUSED(driver_params);

	HAL_Init();
	MX_GPIO_Init();
	MX_DMA_Init(); 
	MX_ADC1_Init(&hadc1);
	MX_ADC2_Init(&hadc2);

	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	MX_DMA1_Init(&hadc1, &hdma_adc1, DMA1_Channel1, DMA_REQUEST_ADC1);
	MX_DMA1_Init(&hadc2, &hdma_adc2, DMA1_Channel2, DMA_REQUEST_ADC2);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer1, ADC_BUF_LEN_1) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("DMA read init failed");
	}
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer2, ADC_BUF_LEN_2) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("DMA read init failed");
	}
	
	Stm32CurrentSenseParams* params = new Stm32CurrentSenseParams {
		.pins = { pinA, pinB, pinC },
		.adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION),
		.timer_handle = ((STM32DriverParams*)driver_params)->timers_handle[0],
	};

	return params;
}

extern "C" {
void DMA1_Channel1_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_adc2);
}
}

void* _driverSyncLowSide(void* _driver_params, void* _cs_params){
	STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
	Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;
	 
	// stop all the timers for the driver
	stm32_pause(driver_params);

	// get the index of the adc
	int adc_index = _adcToIndex(cs_params->adc_handle);

	bool tim_interrupt = _initTimerInterruptDownsampling(cs_params, driver_params, adc_interrupt_config[adc_index]);
	if(tim_interrupt) {
	// error in the timer interrupt initialization
		SIMPLEFOC_DEBUG("STM32-CS: timer has no repetition counter, ADC interrupt not supported for Phoque2");
		return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
	}

	// set the trigger output event
	LL_TIM_SetTriggerOutput(cs_params->timer_handle->Instance, LL_TIM_TRGO_UPDATE);

	// restart all the timers of the driver
	stm32_resume(driver_params);

	// return the cs parameters 
	// successfully initialized
	// TODO verify if success in future
	return _cs_params;
}

extern "C" void initVariant()
{

}

#endif
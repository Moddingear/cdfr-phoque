#if defined(ARDUINO_PHOQUE1)

#include "Phoque1_CurrentSense.hpp"
#include "communication/SimpleFOCDebug.h"
#include "current_sense/hardware_specific/stm32/stm32_adc_utils.h"

static constexpr float adc_voltage = 3.3f;
static constexpr float adc_resolution = 4095.f;

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static OPAMP_HandleTypeDef hopamp1;
static OPAMP_HandleTypeDef hopamp2;
static OPAMP_HandleTypeDef hopamp3;

static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;

Phoque1_CurrentSense::Phoque1_CurrentSense(float _shunt_resistor, float _gain, bool _read_bemf)
	:read_bemf(_read_bemf)
{
	gain_c = gain_b = gain_a = 1.0f /_shunt_resistor / _gain;
	pinA = A_CURRU;
	pinB = A_CURRV;
	pinC = A_CURRW;
	skip_align = true;
}

Phoque1_CurrentSense::Phoque1_CurrentSense(float mVpA, bool _read_bemf)
	:read_bemf(_read_bemf)
{
	gain_c = gain_b = gain_a = 1000.f/mVpA;
	pinA = A_CURRU;
	pinB = A_CURRV;
	pinC = A_CURRW;
	skip_align = true;
}

Phoque1_CurrentSense::~Phoque1_CurrentSense()
{
	if (initialized)
	{
		free((void*)adc1_buffer);
		free((void*)adc2_buffer);
	}
	
}

void Phoque1_CurrentSense::GPIO_Init()
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	__HAL_RCC_ADC12_CLK_ENABLE();
}

void Phoque1_CurrentSense::DMA_Init() 
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

int Phoque1_CurrentSense::ADC1_Init(ADC_HandleTypeDef* hadc1)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config 
	*/
	hadc1->Instance = ADC1;
	hadc1->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
	hadc1->Init.Resolution = ADC_RESOLUTION_12B;
	hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1->Init.GainCompensation = 0;
	hadc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1->Init.LowPowerAutoWait = DISABLE;
	hadc1->Init.ContinuousConvMode = DISABLE;

	hadc1->Init.NbrOfConversion = 3 + read_bemf * 2;
	adc1_buffer = reinterpret_cast<uint16_t*>(malloc(hadc1->Init.NbrOfConversion * sizeof(adc1_buffer[0])));

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
	/** Configure Regular Channel (PA1 / phase U current)
	*/
	sConfig.Channel = _getADCChannel(analogInputToPinName(A_CURRU), ADC1); //ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	if (read_bemf)
	{
		/* Configure Regular Channel (PB0 / BEMFV / Phase V)
		*/
		sConfig.Channel = _getADCChannel(analogInputToPinName(A_BEMFV), ADC1); //ADC_CHANNEL_15;
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
		sConfig.Channel = _getADCChannel(analogInputToPinName(A_BEMFW), ADC1); //ADC_CHANNEL_12;
		sConfig.Rank = ADC_REGULAR_RANK_3;
		sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
		{
			SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
		}
	}

	//******************************************************************
	// Aux analog readings
	/* Configure Regular Channel (PB12, mosfet temperature)
	*/
	sConfig.Channel = _getADCChannel(analogInputToPinName(A_TEMPERATURE), ADC1); //ADC_CHANNEL_11;
	sConfig.Rank = read_bemf ? ADC_REGULAR_RANK_4 : ADC_REGULAR_RANK_2;
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
	sConfig.Channel = _getADCChannel(analogInputToPinName(A_VBUS), ADC1); //ADC_CHANNEL_4;
	sConfig.Rank = read_bemf ? ADC_REGULAR_RANK_5 : ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	return hadc1->Init.NbrOfConversion;
}

int Phoque1_CurrentSense::ADC2_Init(ADC_HandleTypeDef* hadc2)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Common config 
	*/
	hadc2->Instance = ADC2;
	hadc2->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
	hadc2->Init.Resolution = ADC_RESOLUTION_12B;
	hadc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2->Init.GainCompensation = 0;
	hadc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2->Init.LowPowerAutoWait = DISABLE;
	hadc2->Init.ContinuousConvMode = DISABLE;

	hadc2->Init.NbrOfConversion = 3 + read_bemf;
	adc2_buffer = reinterpret_cast<uint16_t*>(malloc(hadc2->Init.NbrOfConversion * sizeof(adc2_buffer[0])));

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
	sConfig.Channel = ADC_CHANNEL_16; // OP2_OUT is ADC2_IN16 for internal channel;
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
	sConfig.Channel = ADC_CHANNEL_18; // OP3_OUT is ADC2_IN18 for internal channel
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}

	if(read_bemf)
	{
		/** Configure Regular Channel (PA6 / BEMF2 / Phase 2)
		*/
		sConfig.Channel = _getADCChannel(analogInputToPinName(A_BEMF2), ADC2);
		sConfig.Rank = ADC_REGULAR_RANK_3;
		sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
		{
			SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
		}

        /** Configure Regular Channel (PC4 / BEMF3 / Phase 3)
		*/
		sConfig.Channel = _getADCChannel(analogInputToPinName(A_BEMF3), ADC2);
		sConfig.Rank = ADC_REGULAR_RANK_4;
		sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
		{
			SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
		}
	}

	/** Configure Regular Channel (PA4 / Potentiometer)
	*/
	sConfig.Channel = _getADCChannel(analogInputToPinName(A_TEMdPERATURE), ADC2); //ADC_CHANNEL_17;
	sConfig.Rank = read_bemf ? ADC_REGULAR_RANK_5 : ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("HAL_ADC_ConfigChannel failed!");
	}
	return hadc2->Init.NbrOfConversion;
}

void Phoque1_CurrentSense::DMA1_Init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef* channel, uint32_t request)
{
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



void* Phoque1_CurrentSense::ConfigureADC(const void* driver_params, const int pinA,const int pinB,const int pinC)
{
	_UNUSED(driver_params);

	HAL_Init();
	GPIO_Init();
	DMA_Init(); 
	int adc1_len = ADC1_Init(&hadc1);
	int adc2_len = ADC2_Init(&hadc2);

	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	DMA1_Init(&hadc1, &hdma_adc1, DMA1_Channel1, DMA_REQUEST_ADC1);
	DMA1_Init(&hadc2, &hdma_adc2, DMA1_Channel2, DMA_REQUEST_ADC2);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, adc1_len) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("ADC1 DMA read init failed");
	}
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, adc2_len) != HAL_OK)
	{
		SIMPLEFOC_DEBUG("ADC2 DMA read init failed");
	}
	
	Stm32CurrentSenseParams* params = new Stm32CurrentSenseParams {
		.pins = { pinA, pinB, pinC },
		.adc_voltage_conv = (adc_voltage) / (adc_resolution),
		.timer_handle = ((STM32DriverParams*)driver_params)->timers_handle[0],
	};

	return params;
}



void* Phoque1_CurrentSense::SyncLowSide(void* _driver_params, void* _cs_params)
{
	STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
	Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;
	 
	// stop all the timers for the driver
	stm32_pause(driver_params);

	//Timer has repetition counter, so this argument is not needed, hence we can let it be garbage
	Stm32AdcInterruptConfig adc_int_config;

	bool tim_interrupt = _initTimerInterruptDownsampling(cs_params, driver_params, adc_int_config);
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

int Phoque1_CurrentSense::init()
{
	if (driver==nullptr) {
		SIMPLEFOC_DEBUG("CUR: Driver not linked!");
		return 0;
	}
	if (initialized)
	{
		return 1;
	}
	

	// configure ADC variables
	params = ConfigureADC(driver->params,pinA,pinB,pinC);
	// if init failed return fail
	if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
	// sync the driver
	_delay(10);
	void* r = SyncLowSide(driver->params, params);
	if(r == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
	// set the center pwm (0 voltage vector)
	if(driver_type==DriverType::BLDC)
		static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit/2, driver->voltage_limit/2, driver->voltage_limit/2);
	// calibrate zero offsets
	_delay(10);
	calibrateOffsets();
	// set zero voltage to all phases
	if(driver_type==DriverType::BLDC)
		static_cast<BLDCDriver*>(driver)->setPwm(0,0,0);
	// set the initialized flag
	initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
	// return success
	return 1;
}

float Phoque1_CurrentSense::readMillivolt(const int pin)
{
	return readRaw(pin) * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv;
}

uint16_t Phoque1_CurrentSense::readRaw(const int pin)
{
    switch (pin)
	{
	case A_CURR3_H:
	case -1:
		return adc2_buffer[1];
	case A_CURR2_H:
	case -2:
		return adc2_buffer[0];
	case A_CURR1_H:
	case -3:
		return adc1_buffer[0];

	case A_BEMF1:
		return adc1_buffer[1];
	case A_BEMF2:
		return adc2_buffer[2];
	case A_BEMF3:
		return adc1_buffer[2];

	case A_POTENTIOMETER:
		return adc1_buffer[2+read_bemf*2];
	case A_TEMPERATURE:
		return adc2_buffer[2+read_bemf];
	case A_VBUS:
		return adc2_buffer[3+read_bemf];
	default:
		return 0;
	}
}

PhaseCurrent_s Phoque1_CurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = (readMillivolt(pinA) - offset_ia)*gain_a; // amps
    current.b = (readMillivolt(pinB) - offset_ib)*gain_b; // amps
    current.c = (readMillivolt(pinC) - offset_ic)*gain_c; // amps
    return current;
}

void Phoque1_CurrentSense::calibrateOffsets(){    
    const int calibration_rounds = 2000;

    // find adc offset = zero current voltage
	uint32_t accA = 0, accB = 0, accC = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
		const uint16_t curru(readRaw(pinA)), currv(readRaw(pinB)), currw(readRaw(pinC));
		//check if new data
		if (curru >= 0x1000 || currv >= 0x1000 || currw >= 0x1000)
		{
			//no new data: this iteration didn't happen
			i--;
			//wait for interrupt (dma irq should do it)
			__WFI();
			continue;
		}
		//invalidate current data
		adc2_buffer[0] = adc2_buffer[1] = adc1_buffer[0] = 0xffff;
        accA += curru;
        accB += currv;
        accC += currw;
    }
    // calculate the mean offsets
    offset_ia = accA * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv / calibration_rounds;
    offset_ib = accB * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv / calibration_rounds;
    offset_ic = accC * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv / calibration_rounds;
}

extern "C" {
void DMA1_Channel1_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_adc2);
}
}

#endif
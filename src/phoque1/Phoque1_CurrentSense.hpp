#pragma once

#if defined(ARDUINO_PHOQUE1)

#include "common/base_classes/CurrentSense.h"

class Phoque1_CurrentSense : public CurrentSense
{
private:
	volatile uint16_t *adc1_buffer = nullptr;
	volatile uint16_t *adc2_buffer = nullptr;
	bool read_bemf;
public:
	Phoque1_CurrentSense(float shunt_resistor, float gain, bool read_bemf=false);
	Phoque1_CurrentSense(float mVpA, bool read_bemf=false);
	~Phoque1_CurrentSense();

	int init() override;
	PhaseCurrent_s getPhaseCurrents() override;

	float readMillivolt(const int pin);
	uint16_t readRaw(const int pin);

private:
	void GPIO_Init();
	void DMA_Init();
    void Opamp_Init();
	int ADC1_Init(ADC_HandleTypeDef* hadc1);
	int ADC2_Init(ADC_HandleTypeDef* hadc2);
	void DMA1_Init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma_adc, DMA_Channel_TypeDef* channel, uint32_t request);
	void* SyncLowSide(void* driver_params, void* cs_params);
	void* ConfigureADC(const void* driver_params, const int pinA, const int pinB, const int pinC);

	void calibrateOffsets();
};

#endif
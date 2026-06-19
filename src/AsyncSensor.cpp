#include "AsyncSensor.hpp"

#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_ll_spi.h"

SPI_HandleTypeDef hspi;

AsyncSensor::AsyncSensor(float (*_decoder)(uint16_t), int _nCS, SPISettings _settings)
	:nCS(digitalPinToPinName(_nCS)), settings(_settings), decoder(_decoder)
{

}

void AsyncSensor::init(SPIClass *_spi)
{
	pinMode(pinNametoDigitalPin(nCS), OUTPUT);
	digitalWriteFast(nCS, HIGH);
	spi = _spi;
	spi->beginTransaction(settings);
	handle = spi->getHandle();
	instance = handle->Instance;

	//keep bitrate, master mode, phase and polarity, and enable spi
	cr1 = (instance->CR1 & (SPI_CR1_BR_Msk | SPI_CR1_MSTR_Msk | SPI_CR1_CPHA_Msk | SPI_CR1_CPOL_Msk));
	cr2 = LL_SPI_DATAWIDTH_16BIT; 
	__WFI();
	__WFI();
	Sensor::init();
}

bool AsyncSensor::start_capture()
{
	if (LL_SPI_IsActiveFlag_BSY(instance) || !cr1 || !cr2)
	{
		return false;
	}
	digitalWriteFast(nCS, LOW);
	instance->CR1 = cr1;
	instance->CR2 = cr2;
	LL_SPI_Enable(instance);
	while (LL_SPI_GetRxFIFOLevel(instance) != LL_SPI_RX_FIFO_EMPTY)
	{
		LL_SPI_ReceiveData8(instance);
	}
	LL_SPI_EnableIT_RXNE(instance);
	LL_SPI_TransmitData16(instance, write_data);
	angle_capture_ts = micros();
	return true;
}

void AsyncSensor::end_capture()
{
	LL_SPI_DisableIT_RXNE(instance);
	digitalWriteFast(nCS, HIGH);
	uint16_t received = LL_SPI_ReceiveData16(instance);
	
	float angle = decoder(received);
	if (angle >= 0)
	{
		capture_index++;
		sensor_data = received;
		decoded_angle = angle;
	}
}

float AsyncSensor::getSensorAngle()
{
	return decoded_angle;
}

void AsyncSensor::update()
{
	while (!has_new_data())
	{
		__WFI();
	}
	last_capture_index = capture_index;
	Sensor::update();
	angle_prev_ts = angle_capture_ts;
}
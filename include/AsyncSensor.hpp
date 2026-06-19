#pragma once

#include "common/base_classes/Sensor.h"
#include <SPI.h>

class AsyncSensor : public Sensor
{
private:
    uint16_t sensor_data;
    float decoded_angle = 0;
    SPI_HandleTypeDef* handle;
    SPI_TypeDef* instance;
    uint32_t cr1, cr2;
    uint8_t capture_index = 0, last_capture_index = 0;
    long angle_capture_ts;
public:
    uint16_t write_data = 0;
    SPIClass* spi;
    SPISettings settings;
    PinName nCS;
    float (*decoder)(uint16_t);

    AsyncSensor(float (*decoder)(uint16_t), int nCS = -1, SPISettings settings = SPISettings());

    bool start_capture();

    void end_capture();

    virtual float getSensorAngle() override;

    virtual void update() override;

    bool has_new_data() const { return capture_index != last_capture_index; }

    void init(SPIClass *_spi = &SPI);
};

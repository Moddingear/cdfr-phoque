#include <foc.hpp>
#include <Arduino.h>
#include <BLDCMotor.h>
#if COMMANDER
#include <communication/Commander.h>
#endif
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>
#ifdef ARDUINO_PHOQUE1
#include <drivers/stspin32g4/STSPIN32G4.h>
#include <current_sense/phoque1/Phoque1_CurrentSense.hpp>
#endif
#if defined(ARDUINO_PHOQUE2) || defined(ARDUINO_PHOQUE2a)
#include <drivers/drv832x/drv832x.h>
#if defined(ARDUINO_PHOQUE2)
#include <current_sense/phoque2/Phoque2_CurrentSense.hpp>
#endif
#if defined(ARDUINO_PHOQUE2a)
#include <current_sense/phoque2a/Phoque2a_CurrentSense.hpp>
#endif
#endif
#if defined(ARDUINO_PHOQUE)
#if BRAKE_EN
#include <brake/phoque/phoque_brake.hpp>
#endif
#include <encoders/sc60228/MagneticSensorSC60228.h>
#include <AsyncSensor.hpp>
#include <stm32g4xx_ll_spi.h>
#endif
#if __has_include("sTune.h")
#define HAS_STUNE
#include "sTune.h"
#endif

#include <current_sense/hardware_specific/stm32/stm32_mcu.h>
#include <stm32g4xx_hal.h>

#include <math.hpp>

#ifdef ARDUINO_PHOQUE2A
#define D4215
#endif

#ifndef D4215
#define POLE_PAIRS 7
#define KV 120
#define PHASE_RESISTANCE (172e-3)
#define PHASE_INDUCTANCE_Q (110e-6)
#define PHASE_INDUCTANCE_D (90e-6)
#define MAX_CURRENT 5
#define MAX_VOLTAGE 20
#define MIN_VOLTAGE 8
#else
#define POLE_PAIRS 7
#define KV 650
#define PHASE_RESISTANCE 0.1
#define PHASE_INDUCTANCE_Q 23e-6
#define PHASE_INDUCTANCE_D 17e-6
#define MAX_CURRENT 5
#define MAX_VOLTAGE 20
#define MIN_VOLTAGE 8
//#define REARM_VOLTAGE 10
#endif

#define RTL 0
#define RTL_ASYNC 0 //Use another timer for the real time loop

#ifdef ARDUINO_PHOQUE2A
#define DEBUG_PINA J7_RX
#define DEBUG_PINB J7_TX
#define DEBUG_PINC I2C_SCL
#endif
#ifdef ARDUINO_PHOQUE2
#define DEBUG_PINA PIN_SERIAL_RX
#define DEBUG_PINB PIN_SERIAL_TX
#define DEBUG_PINC PIN_WIRE_SCL
#endif
#ifdef ARDUINO_PHOQUE1
#define DEBUG_PINA ENCODER_A
#define DEBUG_PINB ENCODER_B
#define DEBUG_PINC ENCODER_Z
#endif

//Phase inductance : dt/ln(V0/V1)*R

// Motor instance
BLDCMotor motor(POLE_PAIRS, PHASE_RESISTANCE, KV, PHASE_INDUCTANCE_Q, PHASE_INDUCTANCE_D);
#ifdef ARDUINO_PHOQUE1
STSPIN32G4 driver;
#ifdef PHOQUE1r2
#warning "Compiling for Phoque1r2"
Phoque1_CurrentSense current_sensor(Phoque1_CurrentSense::compute_equivalent_shunt(3e-3f, 1.5e3f, 22e3f, 2.2e3), 16.f);
#else
Phoque1_CurrentSense current_sensor(Phoque1_CurrentSense::compute_equivalent_shunt(3e-3f, 1.5e3f, 100e3f), -32.f);
#endif
#endif
#if defined(ARDUINO_PHOQUE2) || defined(ARDUINO_PHOQUE2a)
DRV832xDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL, PIN_SS_DRIVER, NOT_SET, NOT_SET, 2000000);
#if defined(ARDUINO_PHOQUE2)
Phoque2_CurrentSense current_sensor(3e-3f, 40.f);
#endif
#if defined(ARDUINO_PHOQUE2a)
Phoque2a_CurrentSense current_sensor(3e-3f, 40.f);
#endif
#endif

#if defined(ARDUINO_PHOQUE2)
SPIClass position_sensor_spi(PIN_MOSI_MAG_ANGLE, PIN_MISO_MAG_ANGLE, PIN_CLK_MAG_ANGLE);
#define MAG_SPI (&position_sensor_spi)
#else
#define MAG_SPI (&SPI)
#endif

#if defined(ARDUINO_PHOQUE)
#ifdef ASYNC_SENSOR
float SC60228_decoder(uint16_t data)
{
	SC60228Angle cast;
	cast.reg = data;
	if (cast.df1 != 1 || cast.df2 != 1 || cast.err == 1)
	{
		return -1;
	}
	
	return (cast.angle/(float)SC60228_CPR)*_2PI;
}
AsyncSensor position_sensor(SC60228_decoder, PIN_SS_MAG_ANGLE, SC60228SPISettings);
#else
MagneticSensorSC60228 position_sensor(PIN_SS_MAG_ANGLE);
#endif
#if BRAKE_EN
phoque_brake brake;
#endif
#endif


#if COMMANDER
Commander commander(Serial);

void OnMotor(char* cmd)
{
	commander.motor(&motor, cmd);
}

#endif

#ifdef HAS_STUNE
float stune_output;
sTune tuner(&motor.target, &stune_output, tuner.NoOvershoot_PID, tuner.directIP, tuner.printALL);

#endif

bool foc_setup = false;

//#define LOGF_CORDIC

#ifdef LOGF_CORDIC
#include <stm32g4xx_ll_cordic.h>
#include "arm_math.h"

uint32_t configure_logf()
{
	LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_NATURALLOG, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0, LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_1, LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
	return CORDIC->CSR;
}

float _logf(float value)
{
	uint32_t csr_before = CORDIC->CSR;
	static uint32_t csr_logf = configure_logf();
	uint8_t scale;
	if (value < 3)
	{
		scale = value < 1 ? 1:2;
	}
	else
	{
		scale = value < 7 ? 3:4;
	}
	
	CORDIC->CSR = csr_logf | (scale << CORDIC_CSR_SCALE_Pos);
	CORDIC->WDATA = (q31_t)((value) * 0x80000000 / (1<<scale));
	q31_t out_log = (int32_t)CORDIC->RDATA;
	CORDIC->CSR = csr_before;
	return (float)out_log * (1<<(scale+1)) / 0x80000000;
}

float Ntc2TempV(uint16_t ADCReading)
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	#if defined(ARDUINO_PHOQUE)
	//B = ln(R/R0)/(1/T-1/T0)
	//V = R/(R+R0)*3.3
	//thus
	float alpha = ADCReading/(4095.f-ADCReading); // R/R0
	float T = 1.f/(_logf(alpha)/NTC_B_CONSTANT + 1.f/NTC_T0);
	return T-273.15f;
	#endif
}
#endif

int FOC::get_temperature()
{
	#ifdef LOGF_CORDIC
	if (current_sensor.initialized)
	{
		return Ntc2TempV(current_sensor.readRaw(A_TEMPERATURE));
	}
	else
	{
		return Ntc2TempV(analogRead(A_TEMPERATURE));
	}
	#else
	if (current_sensor.initialized)
	{
		return current_sensor.read_temperature();
	}
	else
	{
		return current_sensor.get_temperature(analogRead(A_TEMPERATURE));
	}
	#endif
}

constexpr int max_temp = 80, tt_start = 60; //degC

constexpr float get_thermal_throttle_ratio(int temp)
{
	float delta = inverse_lerp<float>(temp, tt_start, max_temp);
	return std::clamp<float>(delta, 0, 1);
}

constexpr float get_max_current(float thermal_throttle)
{
	return lerp<float>(thermal_throttle, MAX_CURRENT, 0.1);
}

constexpr float get_max_voltage(float thermal_throttle)
{
	return lerp<float>(thermal_throttle, MAX_VOLTAGE, 0.1);
}

float FOC::get_vbus()
{
	float vbus_raw;
	if (current_sensor.initialized)
	{
		#if defined(ARDUINO_PHOQUE)
		vbus_raw = current_sensor.readMillivolt(A_VBUS);
		#else
		vbus_raw = _readADCVoltageLowSide(A_VBUS, current_sensor.params);
		#endif
	}
	else
	{
		auto read = analogRead(A_VBUS);
		vbus_raw = read * (3.3f) / ((1<<12)-1);
	}
	#if defined(ARDUINO_PHOQUE)
	return vbus_raw * VBUS_MUL_FACTOR;
	#endif
}

void update_vbus(int temperature)
{
	auto &vbus = driver.voltage_power_supply;

	vbus = FOC::get_vbus();
	float thermal_throttle = get_thermal_throttle_ratio(temperature);
	motor.voltage_limit = min(get_max_voltage(thermal_throttle), vbus/2);
	motor.current_limit = get_max_current(thermal_throttle);
	motor.PID_current_q.limit = motor.voltage_limit;
	motor.PID_current_d.limit = motor.voltage_limit;
}

void param_foc()
{
	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 20;
	driver.pwm_frequency = 20000;
	//driver.voltage_limit = 4;

	motor.voltage_sensor_align = 4*PHASE_RESISTANCE;
	
	// choose FOC modulation (optional)
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	#ifdef D4215
	motor.torque_controller = TorqueControlType::estimated_current;
	#else
	motor.torque_controller = TorqueControlType::estimated_current;
	#endif

	// set motion control loop to be used
	motor.controller = MotionControlType::angle_nocascade;

	motor.modulation_centered = false;

	//motor.PID_velocity.limit *= 10;
	// maximal voltage to be set to the motor
	//motor.voltage_limit = get_max_voltage(0);

	motor.current_limit = get_max_current(0); //A

	//  maximal velocity of the position control
	//motor.velocity_limit = 100;

	motor.motion_downsample = 0;
	motor.monitor_downsample = 1000;
	motor.monitor_decimals = 4;
	motor.P_angle.P = 1;
	current_sensor.skip_align = true;

	motor.PID_current_q.limit = driver.voltage_power_supply;
	motor.PID_current_d.limit = driver.voltage_power_supply;

	//motor.PID_current_d.I = 0;
	//motor.PID_current_q.I = 0;
}

void param_find_pole_pairs()
{
	driver.voltage_power_supply = 12;

	motor.pole_pairs = 1;
}

void init_sensors()
{
	Serial.printf("Starting driver init\n");
	driver.init();

	#if defined(ARDUINO_PHOQUE2) || defined(ARDUINO_PHOQUE2a)
	driver.setRegistersLocked(false);
	driver.setCurrentSenseGain(CSAGain_40);
	driver.setHighSideChargeCurrent(IDRIVEP_1000mA);
	driver.setHighSideDischargeCurrent(IDRIVEN_2000mA);
	driver.setLowSideChargeCurrent(IDRIVEP_1000mA);
	driver.setLowSideDischargeCurrent(IDRIVEN_2000mA);
	driver.setDriveTime(TDRIVE_4000ns);
	#endif

	#if defined(TIM1_DEBUG)
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	LL_TIM_EnableIT_CC4(TIM1);
	pinMode(I2C_SCL, OUTPUT);
	#endif

	#if defined(TIM1_DEBUG) || defined(ASYNC_SENSOR)
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	LL_TIM_EnableIT_UPDATE(TIM1);
	#endif

	#if defined(ARDUINO_PHOQUE1) && 0
	//change fault register to not care about vds protection
	driver.unlock();
	STSPIN32G4NFault faultreg;
	faultreg.reg = 3;
	driver.setNFaultRegister(faultreg);
	driver.lock();
	#endif
	
	#ifdef ASYNC_SENSOR
	SPI.getHandle()->Instance->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE); //disable all interrupts
	HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);
	#endif
	Serial.printf("Starting position sensor init\n");
	position_sensor.init(MAG_SPI);
	pinmap_pinout(digitalPinToPinName(PIN_SPI_MOSI), PinMap_SPI_MOSI);
	LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_LOW);
	// link position sensor
	motor.linkSensor(&position_sensor);

	//link driver
	motor.linkDriver(&driver);
	// link current sense and the driver
	current_sensor.linkDriver(&driver);
	Serial.printf("Starting current sensor init\n");
	current_sensor.init();
	//current_sensor.print_temperature_LUT();
	motor.linkCurrentSense(&current_sensor);


	#ifdef PHOQUE_DEBUG_SAMPLING
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
	pinMode(DEBUG_PINA, OUTPUT);
	pinMode(DEBUG_PINB, OUTPUT);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	#endif
	
	auto temperature = FOC::get_temperature();
	update_vbus(temperature);

	Serial.printf("Updated VBUS to %f, temp is %d\n", driver.voltage_power_supply, temperature);
}

void OnFault()
{
	#if 0
	auto fault = driver.getNFaultRegister();
	auto ready = driver.getReadyRegister();
	Serial.printf("Driver fault detected, fault register is %#x, ready is %#x\n", fault, ready);
	#endif
}

void FOC::setup()
{
	if (FOC::IsSetup())
	{
		return;
	}
	#ifdef HAL_CORDIC_MODULE_ENABLED
    SimpleFOC_CORDIC_Config(); 
	#endif
	#if BRAKE_EN
	brake.init(30);
	#endif
	
	// enable more verbose output for debugging
	// comment out if not needed
	#ifndef SIMPLEFOC_DISABLE_DEBUG
	SimpleFOCDebug::enable(&Serial);
	#endif
	param_foc();
	Serial.printf("param_foc OK\n");
	init_sensors();
	Serial.printf("init_sensors OK\n");
	// initialize motor
	motor.init();
	Serial.printf("motor.init OK\n");
	// init FOC
	bool initFOC_status = motor.initFOC();
	Serial.printf("motor.initFOC %s\n", initFOC_status ? "OK":"KO");

	//motor.tuneCurrentController(5000);
	//motor.characteriseMotor(motor.voltage_sensor_align);
	//motor.tuneCurrentController(5000);

	Serial.printf("Motor ready. Deadzone %f\n", driver.dead_zone);
	
	//FOC::loop();
    //motor.disable();
	//_delay(10000);

	#if COMMANDER
	motor.useMonitoring(Serial);
	motor.monitor_variables = 0;
	char motor_id = 'M';
	commander.add(motor_id, OnMotor, "motor 0");
	//motor.monitor_start_char = motor_id; // the same letter as the motor id in the commander 
	//motor.monitor_end_char = motor_id; // the same letter as the motor id in the commander 
	//commander.verbose = VerboseMode::machine_readable;
	#endif

	#if defined(ARDUINO_PHOQUE1)
	attachInterrupt(PIN_NFAULT, OnFault, RISING);
	#endif
	#if defined(ARDUINO_PHOQUE2) || defined(ARDUINO_PHOQUE2a)
	pinMode(PIN_NFAULT, INPUT);
	#endif

	//pinMode(DEBUG_PINA, OUTPUT);
	//pinMode(DEBUG_PINB, OUTPUT);
	delay(1);
	foc_setup = true;

}

bool FOC::IsSetup()
{
	return foc_setup;
}

float rtl_dt;

void FOC::RealTimeLoop()
{
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINB), HIGH);
	motor.move();
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINB), LOW);
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINA), HIGH);
	motor.loopFOC();
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINA), LOW);

	#if 0
	static float last_shaft_angle = motor.shaftAngle();
	static uint32_t last_time_stamp = micros();
	static float last_speed = 0; //rad/s
	float this_shaft_angle = motor.shaftAngle();
	uint32_t this_timestamp = micros();
	float dt = (this_timestamp - last_time_stamp)/1e6;
	float continuous_shaft_angle = last_shaft_angle+last_speed*dt;
	float delta_angle = this_shaft_angle - continuous_shaft_angle;
	if (abs(delta_angle) > PI/6)
	{
		Serial.printf("ACE! %fr->%fr %fs\n", continuous_shaft_angle, this_shaft_angle, dt);
	}
	rtl_dt = dt;
	
	
	last_speed = (this_shaft_angle-last_shaft_angle)/dt;
	last_shaft_angle = this_shaft_angle;
	last_time_stamp = this_timestamp;
	#endif
}

void FOC::loop()
{
	if (!FOC::IsSetup()) [[unlikely]]
	{
		return;
	}
	static int i = 0;
	static auto i0 = millis();
	auto temp = get_temperature();//1.8us
    update_vbus(temp);//2.25us

	bool motor_fault = false;//0.7us

	//Driver error
	if (digitalReadFast(digitalPinToPinName(PIN_NFAULT)) == LOW) [[unlikely]]
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Driver reported a fault, disabling\n");
		}
	}
	//Over temperature safety
	if (temp > max_temp) [[unlikely]]
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Over temp! %f\n", temp);
		}
	}
	//Undervoltage lockout
	if (driver.voltage_power_supply < MIN_VOLTAGE) [[unlikely]]
	{
		motor_fault = true;
		if (motor.enabled)
		{
			Serial.printf("Voltage too low, disabling\n");
		}
	}
	
	

	if (motor_fault) [[unlikely]]
	{
		if (motor.enabled)
		{
			motor.disable();
		}
	}
	else
	{
		#ifdef REARM_VOLTAGE
		if (driver.voltage_power_supply > REARM_VOLTAGE && !motor.enabled)
		{
			motor.enable();
		}
		#endif
	}
	
	#if RTL
	if (rtl_dt > 2.f/driver.pwm_frequency)
	{
		Serial.printf("RTL took too long! %f\n", rtl_dt);
		rtl_dt = 0;
	}
	#endif
	
	#if defined(ARDUINO_PHOQUE1)
	static auto lastPrint = millis();
	if (millis() - lastPrint > 1000 && !motor.enabled)
	{
		lastPrint = millis();
		auto fault = driver.getNFaultRegister();
		auto ready = driver.getReadyRegister();
		auto power = driver.getPowMngRegister();
		auto status = driver.status();
		Serial.printf("Reporting, fault register is %#x, ready is %#x, power is %#x, status is %#x\n", fault, ready, power, status);
		//driver.clearFaults();
	}
	#endif

	#if (defined(ARDUINO_PHOQUE2) || defined(ARDUINO_PHOQUE2a))
	static auto lastPrint = millis();
	if (millis() - lastPrint > 1000)
	{
		lastPrint = millis();
		auto status = driver.getStatus();
		if (status.isFault())
		{
			Serial.printf("Fault detected %d!\n", millis());
			driver.clearFault();
		}
		if (status.isOverCurrent())
		{
			Serial.printf("Overcurrent reported! Ah%x Al%x Bh%x Bl%x Ch%x Cl%x\n",
				status.isOverCurrent_Ah(),
				status.isOverCurrent_Al(),
				status.isOverCurrent_Bh(),
				status.isOverCurrent_Bl(),
				status.isOverCurrent_Ch(),
				status.isOverCurrent_Cl());
		}
		if (status.isGateDriveFault())
		{
			Serial.printf("Gate drive fault reported! Ah%x Al%x Bh%x Bl%x Ch%x Cl%x\n", 
				status.isGateDriveFault_Ah(),
				status.isGateDriveFault_Al(),
				status.isGateDriveFault_Bh(),
				status.isGateDriveFault_Bl(),
				status.isGateDriveFault_Ch(),
				status.isGateDriveFault_Cl());
		}
		if (status.isUndervoltageLockout())
		{
			Serial.printf("UVLO detected!\n");
		}
		if (status.isOverTemperature())
		{
			Serial.printf("Overtemperature detected!\n");
		}
		if (status.isOverTemperatureWarning())
		{
			Serial.printf("Overtemperature warning!\n");
		}
		if (status.isChargePumpUndervoltage())
		{
			Serial.printf("Charge pump undervoltage detected!\n");
		}
		
		#if 0
		if (!motor.enabled)
		{
			Serial.printf("Brake %d, Coast %d, 1PWMi %d, 1PWMas %d, PWM Mode %d, OTr %d, CPUVr %d\n", driver.isBrake(), driver.isCoast(), driver.isOnePWMInvertDirection(), driver.isOnePWMAsynchronousRectification(), 
				driver.getPWMMode(), driver.isOvertemperatureReporting(), driver.isDisableGateDriveFault(), driver.isDisableChargePumpUndervoltage());
			Serial.printf("VDS %d, OCP %d, tOCP %d, dt %d\n", driver.getVDSLevel(), driver.getOCPMode(), driver.getOCPDeglitchTime(), driver.getDeadTime());
			Serial.printf("TDrive %d, Currents : HN %d HP %d LN %d LP%d\n", driver.getDriveTime(), driver.getHighSideDischargeCurrent(), driver.getHighSideChargeCurrent(), driver.getLowSideDischargeCurrent(), driver.getLowSideChargeCurrent());
			Serial.printf("Current sense Gain : %d\n", driver.getCurrentSenseGain());
		}
		#endif
		
		//driver.clearFaults();
	}
	#endif
	
#if 1
	#if 0  //Print angle every 100 millis
	static auto lastPrint = millis();
	if (millis() - lastPrint > 100)
	{
		lastPrint = millis();
		Serial.printf("Angle %f, Hall type %" PRIx8 "\n", position_sensor.getAngle()*180.f/M_PI, position_sensor.hall_type);
	}
	#endif
	#if 0 //Print angle every time it changes
	static auto lastAngle = position_sensor.getAngle();
	if (lastAngle != position_sensor.getAngle())
	{
		if (!motor.enabled)
		{
			position_sensor.update();
		}
		
		lastAngle = position_sensor.getAngle();
		//int hall_type = (int)position_sensor.hall_type;
		Serial.printf("Angle %f\n", position_sensor.getAngle()*180.f/M_PI);
	}
	
	#endif
	#if 0 //Print phase currents
	PhaseCurrent_s currents = current_sensor.getPhaseCurrents();
	float current_magnitude = current_sensor.getDCCurrent();

	Serial.printf("%f, %f, %f, %f\n", currents.a, currents.b, currents.c, current_magnitude);
	#endif
#endif

	#if COMMANDER
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINA), HIGH);
	motor.monitor();
	commander.run();
	//digitalWriteFast(digitalPinToPinName(DEBUG_PINA), LOW);
	#endif
    
	// main FOC algorithm function
	#if !RTL
	#if ASYNC_SENSOR
	/*if (position_sensor.has_new_data())
	{
		RealTimeLoop();
	}*/
	#else
	RealTimeLoop();
	#endif
	#if 0
	if (millis() - i0 > 10000)
	{
		auto time_now = millis();
		float loop_frequency = (float)i/(time_now-i0)*1000;
		i = 0;
		Serial.printf("FOC loop frequency is %fHz\n", loop_frequency);
		i0 = millis();
	}
	else
	{
		i++;
	}
	#endif
	
	#endif
}

void FOC::start()
{
    motor.enable();
}

void FOC::stop()
{
    motor.disable();
}

void FOC::set_target(float InTarget)
{
	motor.move(InTarget);
}

void FOC::set_motion_mode(MotionControlType type)
{
	motor.controller = type;
}

float FOC::get_angle()
{
	return motor.shaftAngle();
}

float FOC::get_current()
{
	return motor.current.q;
}

void FOC::tune_settings(const RobotCAN::PIDTunePacket *message)
{
	PIDController* controller = nullptr;
	LowPassFilter* lpf = nullptr;
	switch (message->loop)
	{
	case RobotCAN::PIDTunePacket::loop_select::current_d :
		controller = &motor.PID_current_d;
		lpf = &motor.LPF_current_d;
		break;
	case RobotCAN::PIDTunePacket::loop_select::current_q :
		controller = &motor.PID_current_q;
		lpf = &motor.LPF_current_q;
		break;
	case RobotCAN::PIDTunePacket::loop_select::velocity :
		controller = &motor.PID_velocity;
		lpf = &motor.LPF_velocity;
		break;
	case RobotCAN::PIDTunePacket::loop_select::angle :
		controller = &motor.P_angle;
		lpf = &motor.LPF_angle;
		break;
	default:
		break;
	}
	switch (message->parameter)
	{
	case RobotCAN::PIDTunePacket::parameter_select::P :
		controller->P = message->new_value;
		break;
	case RobotCAN::PIDTunePacket::parameter_select::I :
		controller->I = message->new_value;
		break;
	case RobotCAN::PIDTunePacket::parameter_select::D :
		controller->D = message->new_value;
		break;
	case RobotCAN::PIDTunePacket::parameter_select::Limit :
		controller->limit = message->new_value;
		break;
	case RobotCAN::PIDTunePacket::parameter_select::Ramp :
		controller->output_ramp = message->new_value;
		break;
	case RobotCAN::PIDTunePacket::parameter_select::LPF :
		lpf->Tf = message->new_value;
		break;
	
	default:
		break;
	}
}

extern "C" {
	#if defined(TIM1_DEBUG) || defined(ASYNC_SENSOR)
	void TIM1_UP_TIM16_IRQHandler()
	{
		if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
		{
			#ifdef ASYNC_SENSOR
			position_sensor.start_capture();
			#endif
			#ifdef TIM1_DEBUG
			HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINC), digitalPinToBitMask(DEBUG_PINC), GPIO_PIN_RESET);
			#endif
			LL_TIM_ClearFlag_UPDATE(TIM1);
			
		}
	}
	#endif

	#ifdef TIM1_DEBUG
	void TIM1_CC_IRQHandler()
	{
		if (LL_TIM_IsActiveFlag_CC4(TIM1))
		{
			HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINC), digitalPinToBitMask(DEBUG_PINC), GPIO_PIN_SET);
			LL_TIM_ClearFlag_CC4(TIM1);
		}
	}
	#endif

	#ifdef PHOQUE_DEBUG_SAMPLING
	void ADC1_2_IRQHandler()
	{
		auto adc1_isr = ADC1->ISR;
		if (adc1_isr)
		{
			if (adc1_isr & ADC_ISR_EOSMP)
			{
				HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINA), digitalPinToBitMask(DEBUG_PINA), GPIO_PIN_SET);
			}
			if (adc1_isr & ADC_ISR_EOS)
			{
				HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINA), digitalPinToBitMask(DEBUG_PINA), GPIO_PIN_RESET);
			}
			
			ADC1->ISR = adc1_isr;
		}
		auto adc2_isr = ADC2->ISR;
		if (adc2_isr)
		{
			if (adc2_isr & ADC_ISR_EOSMP)
			{
				HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINB), digitalPinToBitMask(DEBUG_PINB), GPIO_PIN_SET);
			}
			if (adc2_isr & ADC_ISR_EOS)
			{
				HAL_GPIO_WritePin(digitalPinToPort(DEBUG_PINB), digitalPinToBitMask(DEBUG_PINB), GPIO_PIN_RESET);
			}
			ADC2->ISR = adc2_isr;
		}
	}
	#endif

	#ifdef ASYNC_SENSOR
	void SPI1_IRQHandler()
	{
		SPI_TypeDef *instance = position_sensor.spi->getHandle()->Instance;
		if (LL_SPI_IsActiveFlag_RXNE(instance))
		{
			position_sensor.end_capture();
			if (FOC::IsSetup())
			{
				FOC::RealTimeLoop();
			}
		}
	}
	#endif
}
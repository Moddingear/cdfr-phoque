#include <foc.hpp>
#include <Arduino.h>
#include <BLDCMotor.h>
#include <communication/Commander.h>
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>
#if PHOQUE
#include <drivers/stspin32g4/STSPIN32G4.h>
#include <encoders/sc60228/MagneticSensorSC60228.h>
#else
#include <encoders/as5600/MagneticSensorAS5600.h>
#include <drivers/BLDCDriver6PWM.h>
#include <current_sense/LowsideCurrentSense.h>
#endif
#include <current_sense/hardware_specific/stm32/stm32_mcu.h>

#include <math.hpp>

#define FIND_PP 0

#define D4215 0

#if D4215
#define POLE_PAIRS 7
#define KV 650
#define PHASE_RESISTANCE (85e-3*1.5)
#define PHASE_INDUCTANCE NOT_SET
#define HALL_SENSOR 0
#define MAX_CURRENT 1
#define MAX_VOLTAGE 1
#else
#define POLE_PAIRS 7
#define KV 120
#define PHASE_RESISTANCE (160e-3*1.5)
#define PHASE_INDUCTANCE (173e-6*1.5)
#define HALL_SENSOR 0
#define MAX_CURRENT 40
#define MAX_VOLTAGE 20
#endif

#define RTL 0

//Phase inductance : dt/ln(V0/V1)*R

// Motor instance
BLDCMotor motor(FIND_PP ? 1 : POLE_PAIRS, PHASE_RESISTANCE, KV, PHASE_INDUCTANCE);
#if PHOQUE
#if 1
STSPIN32G4 driver;
#else 
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
#endif
#else
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense current_sensor(-(1.714*4.f), A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
#endif

#if HALL_SENSOR
HallSensor position_sensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);
void doA(){position_sensor.handleA();}
void doB(){position_sensor.handleB();}
void doC(){position_sensor.handleC();}
#else
#if PHOQUE
SPIClass magspi(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
MagneticSensorSC60228 position_sensor(PIN_SS_MAG_ANGLE);
#else
#if 1
MagneticSensorAS5600 position_sensor;
#else
MagneticSensorI2C position_sensor(AS5600_I2C);
#endif
#endif
#endif

#if STANDALONE
Commander commander(Serial);

void OnMotor(char* cmd)
{
	commander.motor(&motor, cmd);
}

#endif

bool foc_setup = false;

constexpr float Ntc2TempV(float ADCVoltage)
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F/298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;
	
	float T = 1.0F/((logf(Rt/R25)/Beta)+RoomTempI);
	T = T - 273.15;

	return T;
}

float FOC::get_temperature()
{	
	#if PHOQUE
	return 0;
	#else
	return Ntc2TempV(_readADCVoltageLowSide(A_TEMPERATURE, current_sensor.params));
	#endif
}

constexpr float max_temp = 80, tt_start = 60; //degC

constexpr float get_max_current(float temp)
{
	float delta = inverse_lerp<float>(temp, tt_start, max_temp);
	delta=std::clamp<float>(delta, 0, 1);
	return lerp<float>(delta, MAX_CURRENT, 0.1);
}

constexpr float get_max_voltage(float temp)
{
	float delta = inverse_lerp<float>(temp, tt_start, max_temp);
	delta=std::clamp<float>(delta, 0, 1);
	return lerp<float>(delta, MAX_VOLTAGE, 0.1);
}

float FOC::get_vbus()
{
	float vbus_raw;
	#if !PHOQUE
	if (IsSetup())
	{
		vbus_raw = _readADCVoltageLowSide(A_VBUS, current_sensor.params);
	}
	else
	#endif
	{
		vbus_raw = analogRead(A_VBUS) * (3.3f) / (1<<12);
	}
	#if PHOQUE
	return vbus_raw * (220+10)/10.f;
	#else
	return vbus_raw * (169+18)/18.f;
	#endif
}

void update_vbus()
{
	auto &vbus = driver.voltage_power_supply;

	vbus = FOC::get_vbus();
	motor.voltage_limit = min(get_max_voltage(FOC::get_temperature()), vbus/2);
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

	motor.torque_controller = TorqueControlType::voltage;

	// set motion control loop to be used
	motor.controller = MotionControlType::torque;

	motor.modulation_centered = true;

	motor.PID_velocity.limit *= 10;
	// maximal voltage to be set to the motor
	//motor.voltage_limit = get_max_voltage(0);

	motor.current_limit = get_max_current(0); //A

	//  maximal velocity of the position control
	//motor.velocity_limit = 100;

	motor.motion_downsample = 0;
	motor.monitor_downsample = 1000;
	motor.monitor_decimals = 4;
	#if !PHOQUE
	current_sensor.skip_align = true;
	position_sensor.closeTransactions = false;
	#endif

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
	driver.init();
	#if PHOQUE && 0
	//change gate drive voltage
	driver.unlock();
	auto powmng = driver.getPowMngRegister();
	powmng.vcc_val = 0b10;
	driver.setPowMngRegister(powmng);
	driver.lock();
	#endif
	#if PHOQUE && 0
	//change fault register to not care about vds protection
	driver.unlock();
	STSPIN32G4NFault faultreg;
	faultreg.reg = 3;
	driver.setNFaultRegister(faultreg);
	driver.lock();
	#endif
	//digitalWrite(PE7, HIGH);
	//driver.clearFaults();
	#if PHOQUE
	position_sensor.init(&magspi);
	#else
	position_sensor.init();
	#endif
	#if HALL_SENSOR
	position_sensor.enableInterrupts(doA, doB, doC); //only for hall
	#else
	#if PHOQUE
	
	#else
	Wire.setClock(400000UL);
	#endif
	#endif
	// link position sensor
	motor.linkSensor(&position_sensor);

	//link driver
	motor.linkDriver(&driver);
	// link current sense and the driver
	#if !PHOQUE
	current_sensor.linkDriver(&driver);
	current_sensor.init();
	motor.linkCurrentSense(&current_sensor);
	#endif

	update_vbus();

}

int find_pole_pairs()
{
	// pole pairs calculation routine
	Serial.println("Pole pairs (PP) estimator");
	Serial.println("-\n");

	float pp_search_voltage = 1; // maximum power_supply_voltage/2
	float pp_search_angle = 20*_PI; // search electrical angle to turn

	// move motor to the electrical angle 0
	motor.controller = MotionControlType::angle_openloop;
	motor.voltage_limit=pp_search_voltage;
	motor.move(0);
	_delay(1000);
	// read the sensor angle
	position_sensor.update();
	float angle_begin = position_sensor.getAngle();
	_delay(50);

	// move the motor slowly to the electrical angle pp_search_angle
	float motor_angle = 0;
	while(motor_angle <= pp_search_angle){
		motor_angle += 0.01f;
		position_sensor.update(); // keep track of the overflow
		motor.move(motor_angle);
		_delay(1);
	}
	_delay(1000);
	// read the sensor value for 180
	position_sensor.update(); 
	float angle_end = position_sensor.getAngle();
	_delay(50);
	// turn off the motor
	motor.move(0);
	_delay(1000);

	// calculate the pole pair number
	int pp = round((pp_search_angle)/(angle_end-angle_begin));

	Serial.print(F("Estimated PP : "));
	Serial.println(pp);
	Serial.println(F("PP = Electrical angle / Encoder angle "));
	Serial.print(pp_search_angle*180/_PI);
	Serial.print(F("/"));
	Serial.print((angle_end-angle_begin)*180/_PI);
	Serial.print(F(" = "));
	Serial.println((pp_search_angle)/(angle_end-angle_begin));
	Serial.println();


	// a bit of monitoring the result
	if(pp <= 0 ){
		Serial.println(F("PP number cannot be negative"));
		Serial.println(F(" - Try changing the search_voltage value or motor/sensor configuration."));
	}else if(pp > 30){
		Serial.println(F("PP number very high, possible error."));
	}else{
		Serial.println(F("If PP is estimated well your motor should turn now!"));
		Serial.println(F(" - If it is not moving try to relaunch the program!"));
		Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
	}
	motor.disable();
	return pp;
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
	
    SimpleFOC_CORDIC_Config(); 
	
	// enable more verbose output for debugging
	// comment out if not needed
	SimpleFOCDebug::enable(&Serial);
	#if FIND_PP
	param_find_pole_pairs();
	#else
	param_foc();
	Serial.printf("param_foc OK\n");
	#endif
	init_sensors();
	Serial.printf("init_sensors OK\n");
	#if FIND_PP
	auto pairs = find_pole_pairs();
	#else
	// initialize motor
	motor.init();
	Serial.printf("motor.init OK\n");
	// init FOC
	foc_setup = motor.initFOC();
	Serial.printf("motor.initFOC %s\n", foc_setup ? "OK":"KO");
	foc_setup = true;
	#endif

	Serial.printf("Motor ready. Deadzone %f\n", driver.dead_zone);

	#if HALL_SENSOR
	Serial.printf("Hall sensor type %d\n", position_sensor.hall_type);
	#else
	#if PHOQUE
	
	#else
	auto status = position_sensor.readStatus();
	Serial.printf("Magnetic sensor detected %d, too weak %d, too strong %d, magnitude %d\n", status.md, status.ml, status.mh, position_sensor.readMagnitude());
	#endif
	#endif
	
	//FOC::loop();
    motor.disable();
	//_delay(10000);

	#if STANDALONE
	motor.useMonitoring(Serial);
	motor.monitor_variables = 0;
	char motor_id = 'M';
	commander.add(motor_id, OnMotor, "motor 0");
	//motor.monitor_start_char = motor_id; // the same letter as the motor id in the commander 
	//motor.monitor_end_char = motor_id; // the same letter as the motor id in the commander 
	//commander.verbose = VerboseMode::machine_readable;
	#endif

	#if PHOQUE
	attachInterrupt(PIN_NFAULT, OnFault, RISING);
	#endif

	#if RTL
	HardwareTimer* timer = new HardwareTimer(TIM4);
	//auto driver_params = reinterpret_cast<STM32DriverParams*>(driver.params);
	timer->setOverflow(driver.pwm_frequency, HERTZ_FORMAT); 
	// add the loopFOC and move to the timer
	timer->attachInterrupt(&FOC::RealTimeLoop);
	timer->setInterruptPriority(1, 0);
	// start the timer
	timer->resume();
	#endif

}

bool FOC::IsSetup()
{
	return foc_setup;
}

float rtl_dt;

void FOC::RealTimeLoop()
{
	motor.loopFOC();
	motor.move();

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
	static int i = 0;
	static auto i0 = millis();
	if (!FOC::IsSetup())
	{
		return;
	}
	
	auto temp = get_temperature();
	#if 0
    update_vbus();
	motor.current_limit = get_max_current(temp);
	#endif

	if (temp > max_temp && motor.enabled)
	{
		motor.disable();
		Serial.printf("Over temp! %f\n", temp);
	}

	if (get_vbus() < 8 && motor.enabled)
	{
		motor.disable();
		Serial.printf("Voltage too low, disabling\n");
	}

	#if RTL
	if (rtl_dt > 2.f/driver.pwm_frequency)
	{
		Serial.printf("RTL took too long! %f\n", rtl_dt);
		rtl_dt = 0;
	}
	#endif

	#if PHOQUE
	static bool last_enabled = true;
	if (!motor.enabled && motor.enabled != last_enabled)
	{
		Serial.printf("Enabling low sides\n");
		driver.setPhaseState(PHASE_ON, PHASE_ON, PHASE_ON);
		driver.setPwm(0,0,0);
		driver.clearFaults();
	}
	last_enabled = motor.enabled;
	

	#endif
	
	

	#if PHOQUE && 0
	static auto lastPrint = millis();
	if (millis() - lastPrint > 1000)
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

	#if STANDALONE
	motor.monitor();
	commander.run();
	#endif
    
	// main FOC algorithm function
	#if !RTL
	RealTimeLoop();

	#if 1
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
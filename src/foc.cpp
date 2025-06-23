#include <foc.hpp>
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>
#include <encoders/as5600/MagneticSensorAS5600.h>
#include <current_sense/hardware_specific/stm32/stm32_mcu.h>

#include <math.hpp>

#define FIND_PP 0

#define D4215 1

#if D4215
#define POLE_PAIRS 7
#define KV 650
#define PHASE_RESISTANCE (85e-3*1.5)
#define PHASE_INDUCTANCE NOT_SET
#define HALL_SENSOR 0
#else
#define POLE_PAIRS 7
#define KV 120
#define PHASE_RESISTANCE (160e-3*1.5)
#define PHASE_INDUCTANCE (173e-6*1.5)
#define HALL_SENSOR 1
#endif

//Phase inductance : dt/ln(V0/V1)*R

// Motor instance
BLDCMotor motor(FIND_PP ? 1 : POLE_PAIRS, PHASE_RESISTANCE, KV, PHASE_INDUCTANCE);
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense current_sensor(-(1.714*4.f), A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

#if HALL_SENSOR
HallSensor position_sensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);
void doA(){position_sensor.handleA();}
void doB(){position_sensor.handleB();}
void doC(){position_sensor.handleC();}
#else
#if 1
MagneticSensorAS5600 position_sensor;
#else
MagneticSensorI2C position_sensor(AS5600_I2C);
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
	return Ntc2TempV(_readADCVoltageLowSide(A_TEMPERATURE, current_sensor.params));
}

constexpr float max_temp = 80, tt_start = 60; //degC

void update_vbus()
{
	auto &vbus = driver.voltage_power_supply;
	vbus = _readADCVoltageLowSide(A_VBUS, current_sensor.params) * (169+18)/18.f;
	motor.voltage_limit = vbus/2;
	motor.PID_current_q.limit = motor.voltage_limit;
	motor.PID_current_d.limit = motor.voltage_limit;
}

constexpr float get_max_current(float temp)
{
	float delta = inverse_lerp<float>(temp, tt_start, max_temp);
	delta=std::clamp<float>(delta, 0, 1);
	return lerp<float>(delta, 40.f, 0.1);
}

constexpr float get_max_voltage(float temp)
{
	float delta = inverse_lerp<float>(temp, tt_start, max_temp);
	delta=std::clamp<float>(delta, 0, 1);
	return lerp<float>(delta, 10.f, 0.1);
}

void param_foc()
{
	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 20;
	//driver.pwm_frequency = 100000;
	//driver.voltage_limit = 4;

	motor.voltage_sensor_align = 4*PHASE_RESISTANCE;
	
	// choose FOC modulation (optional)
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	motor.torque_controller = TorqueControlType::foc_current;

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
	position_sensor.init();
	#if HALL_SENSOR
	position_sensor.enableInterrupts(doA, doB, doC); //only for hall
	#else
	Wire.setClock(1000000UL);
	#endif
	// link position sensor
	motor.linkSensor(&position_sensor);

	driver.init();
	//link driver
	motor.linkDriver(&driver);
	// link current sense and the driver
	current_sensor.linkDriver(&driver);
	current_sensor.init();
	motor.linkCurrentSense(&current_sensor);

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
	#endif

	Serial.println(F("Motor ready."));

	#if HALL_SENSOR
	Serial.printf("Hall sensor type %d\n", position_sensor.hall_type);
	#else
	auto status = position_sensor.readStatus();
	Serial.printf("Magnetic sensor detected %d, too weak %d, too strong %d, magnitude %d\n", status.md, status.ml, status.mh, position_sensor.readMagnitude());
	#endif
	
	//FOC::loop();
    motor.disable();
	//_delay(10000);

	#if STANDALONE
	motor.useMonitoring(Serial);
	char motor_id = 'M';
	commander.add(motor_id, OnMotor, "motor 0");
	motor.monitor_start_char = motor_id; // the same latter as the motor id in the commander 
	motor.monitor_end_char = motor_id; // the same latter as the motor id in the commander 
	commander.verbose = VerboseMode::machine_readable;
	#endif
}

bool FOC::IsSetup()
{
	return foc_setup;
}

void FOC::loop()
{
	if (!FOC::IsSetup())
	{
		return;
	}
	
    update_vbus();
	auto temp = get_temperature();
	motor.current_limit = get_max_current(temp);

	if (temp > max_temp)
	{
		motor.disable();
        //todo : throw over temp error
	}
	
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
		lastAngle = position_sensor.getAngle();
		int hall_type = (int)position_sensor.hall_type;
		Serial.printf("Angle %f, hall type %x\n", position_sensor.getAngle()*180.f/M_PI, hall_type);
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
	motor.loopFOC();
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
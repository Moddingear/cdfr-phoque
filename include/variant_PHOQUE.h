#pragma once



/* Alias */
#define A_PHASE_UL				PE8
#define A_PHASE_UH				PE9
#define A_PHASE_VL				PE10
#define A_PHASE_VH				PE11
#define A_PHASE_WL				PE12
#define A_PHASE_WH				PE13

#define PIN_READY				PE14	//Gate driver ready pin
#define PIN_NFAULT				PE15	//Gate driver fault pin
#define PIN_I2C_GD_SCL			PC8		//I2C3 connected to Gate driver
#define PIN_I2C_GD_SDA			PC9		//I2C3 connected to Gate driver

#define A_VBUS					PC0		//Pin to sense supply voltage

#define VBUS_MUL_FACTOR			23		//Multiply the adc voltage read by this value to get the supply voltage

#define A_GPIO_BEMF				PA4		//Pin to pull Back-emf high or low

#define A_CURR1_H				PA1		//CS1 connected to phase W and opamp1
#define A_CURR1_L				PA3
#define A_BEMF1					PA2

#define A_CURR2_H				PA7		//CS2 connected to phase V and opamp 2
#define A_CURR2_L				PA5
#define A_BEMF2					PC5

#define A_CURR3_H				PB0		//CS3 connected to phase U and opamp 3
#define A_CURR3_L				PB2
#define A_BEMF3					PB1

#define A_POTENTIOMETER			PC1
#define A_TEMPERATURE			PF1
#define A_TEMP_EXT				PF0

#define NTC_B_CONSTANT			3455	//B = ln(R/R0)/(1/T-1/T0)
#define NTC_T0					(273+25)//Kelvin

#define A_CAN_RX				PB8
#define A_CAN_TX				PB9
#define A_CAN_SHDN				PB7

#define A_SWDIO					PA13
#define A_SWCLK					PA14

#define A_I2C2SCL_TX1			PA9
#define A_I2C2SDA				PA8
#define A_RX1					PA10

#define A_PWM					A_I2C2SDA		//Legacy PWM input for compat with BG431B-ESC1
#define A_USART_TX				A_I2C2SCL_TX1
#define A_USART_RX				A_RX1

#define A_SPARE1				PC2
#define A_SPARE2				PA0
#define A_SPARE3				PC3

#define A_HALL1					A_SPARE1
#define A_HALL2					A_SPARE2
#define A_HALL3					A_SPARE3

#define A_ENCODER_A				A_SPARE1
#define A_ENCODER_B				A_SPARE3
#define A_ENCODER_Z				A_SPARE2

#define A_USBCPD_CC1			PB4
#define A_USBCPD_CC2			PB6

// On-board LED pin number
#define LED_RED					PC13
#ifndef LED_BUILTIN
	#define LED_BUILTIN				LED_RED
#endif

#define PIN_PWM_MAG_ANGLE
#define PIN_SS_MAG_ANGLE		PA15

// SPI definitions
#ifndef PIN_SPI_SS
	#define PIN_SPI_SS				PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS1
	#define PIN_SPI_SS1				PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS2
	#define PIN_SPI_SS2				PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS3
	#define PIN_SPI_SS3				PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_MOSI
	#define PIN_SPI_MOSI			PB5
#endif
#ifndef PIN_SPI_MISO
	#define PIN_SPI_MISO			PA6
#endif
#ifndef PIN_SPI_SCK
	#define PIN_SPI_SCK				PB3
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
	#define PIN_WIRE_SDA			A_I2C2SDA
#endif
#ifndef PIN_WIRE_SCL
	#define PIN_WIRE_SCL			A_I2C2SCL_TX1
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
	#define TIMER_TONE			TIM6
#endif
#ifndef TIMER_SERVO
	#define TIMER_SERVO			TIM7
#endif

// UART Definitions
#define SERIAL_UART_INSTANCE	1 //Connected to ST-Link

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
	#define PIN_SERIAL_RX			A_USART_RX
#endif
#ifndef PIN_SERIAL_TX
	#define PIN_SERIAL_TX			A_USART_TX
#endif

/* HAL configuration */
//#define HSE_VALUE				(8000000UL)

/* Extra HAL modules */
#if !defined(HAL_DAC_MODULE_DISABLED)
	#define HAL_DAC_MODULE_ENABLED
#endif
#if !defined(HAL_FDCAN_MODULE_DISABLED)
	#define HAL_FDCAN_MODULE_ENABLED
#endif
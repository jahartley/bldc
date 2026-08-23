/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_CLASSICP_CORE_H_
#define HW_CLASSICP_CORE_H_

#ifdef HWCLASSICP
	#define HW_NAME					"Classicp"
#else
	#error "Must define hardware type"
#endif

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_FILTERS
#define HW_HAS_PHASE_SHUNTS

// Macros
#define LED_GREEN_GPIO			GPIOC
#define LED_GREEN_PIN			9
#define LED_RED_GPIO			GPIOC
#define LED_RED_PIN				12

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PHASE_FILTER_OFF()		palClearPad(GPIOB, 12)
#define PHASE_FILTER_ON()		palSetPad(GPIOB, 12)

#define CURRENT_FILTER_GPIO		GPIOC
#define CURRENT_FILTER_PIN		15
#define CURRENT_FILTER_ON()		palSetPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)
#define CURRENT_FILTER_OFF()	palClearPad(CURRENT_FILTER_GPIO, CURRENT_FILTER_PIN)

#define AUX_GPIO				GPIOC
#define AUX_PIN					14
#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)

#define HW_SHUTDOWN_HOLD_ON();
#define HW_SAMPLE_SHUTDOWN()		1
#define HW_SAMPLE_SHUTDOWN_OVR()	smart_switch_is_pressed()
#define HW_SHUTDOWN_HOLD_OFF()		palClearPad(SWITCH_OUT_GPIO, SWITCH_OUT_PIN);
#define HW_SHUTDOWN_NO

#define DCCAL_ON()
#define DCCAL_OFF()

#define HW_EARLY_INIT()				smart_switch_pin_init(); \
									smart_switch_thread_start();

#define SMART_SWITCH_MSECS_PRESSED_OFF		1500

#define SWITCH_OUT_GPIO				GPIOB
#define SWITCH_OUT_PIN				2
#define SWITCH_LED_3_GPIO			GPIOD
#define SWITCH_LED_3_PIN			2
#define SWITCH_LED_2_GPIO			GPIOC
#define SWITCH_LED_2_PIN			13
#define SWITCH_LED_1_GPIO			GPIOB
#define SWITCH_LED_1_PIN			7

#define LED_PWM1_ON()			palClearPad(SWITCH_LED_1_GPIO,SWITCH_LED_1_PIN)
#define LED_PWM1_OFF()			palSetPad(SWITCH_LED_1_GPIO,SWITCH_LED_1_PIN)
#define LED_PWM2_ON()			palClearPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_PWM2_OFF()			palSetPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_PWM3_ON()			palClearPad(SWITCH_LED_3_GPIO, SWITCH_LED_3_PIN)
#define LED_PWM3_OFF()			palSetPad(SWITCH_LED_3_GPIO, SWITCH_LED_3_PIN)

#define LED_SWITCH_R_ON()			palClearPad(SWITCH_LED_3_GPIO,SWITCH_LED_3_PIN)
#define LED_SWITCH_R_OFF()			palSetPad(SWITCH_LED_3_GPIO,SWITCH_LED_3_PIN)
#define LED_SWITCH_G_ON()			palClearPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_SWITCH_G_OFF()			palSetPad(SWITCH_LED_2_GPIO, SWITCH_LED_2_PIN)
#define LED_SWITCH_B_ON()			palClearPad(SWITCH_LED_1_GPIO, SWITCH_LED_1_PIN)
#define LED_SWITCH_B_OFF()			palSetPad(SWITCH_LED_1_GPIO, SWITCH_LED_1_PIN)

/*
 * ADC Vector
 */
#define HW_ADC_NBR_CONV			7
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)
#define HW_ADC_INJ_CHANNELS		3

// ADC Indicies
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_VIN_SENS		14
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT4			12
#define ADC_IND_EXT5			13
#define ADC_IND_TEMP_MOS		18
#define ADC_IND_TEMP_MOS_2		19
#define ADC_IND_TEMP_MOTOR		16
#define ADC_IND_SW_DET			15

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					150000.0
#endif
#ifndef VIN_R2
#define VIN_R2					4700.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.00025 / 2.0)
#endif

#define ENCODER_SIN_VOLTS		ADC_VOLTS(ADC_IND_EXT4)
#define ENCODER_COS_VOLTS		ADC_VOLTS(ADC_IND_EXT5)

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP(adc_ind)		hw_classicp_get_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()			NTC_TEMP_MOS2()

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			7
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6
#define HW_ADC_EXT4_GPIO		GPIOC
#define HW_ADC_EXT4_PIN			4
#define HW_ADC_EXT5_GPIO		GPIOB
#define HW_ADC_EXT5_PIN			1

// --- JAH added KILL UART TO REUSE PINS, POISON THE DEFINES AT BOTTOM OF THIS FILE.
// UART Peripheral
// #define HW_UART_DEV				SD3
// #define HW_UART_GPIO_AF			GPIO_AF_USART3
// #define HW_UART_TX_PORT			GPIOB
// #define HW_UART_TX_PIN			10
// #define HW_UART_RX_PORT			GPIOB
// #define HW_UART_RX_PIN			11

// --- JAH added WRSM H-BRIDGE ROTOR EXCITAL CONTROLS ---
#define HW_FIELD_PWM_GPIO             GPIOB
#define HW_FIELD_PWM_PIN              6
#define HW_FIELD_PWM_AF               GPIO_AF_TIM4

#define HW_FIELD_EN_GPIO              GPIOB
#define HW_FIELD_EN_PIN               10

// Permanent UART Peripheral (SWD/ESP)
// TODO: Encoder UART
//#define HW_UART_P_BAUD			115200
//#define HW_UART_P_DEV			SD4
//#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
//#define HW_UART_P_TX_PORT		GPIOC
//#define HW_UART_P_TX_PIN		10
//#define HW_UART_P_RX_PORT		GPIOC
//#define HW_UART_P_RX_PIN		11

// --- JAH added KILL ICU TO REUSE PINS, POISON THE DEFINES AT BOTTOM OF THIS FILE.
// ICU Peripheral for servo decoding
// #define HW_USE_SERVO_TIM4
// #define HW_ICU_TIMER			TIM4
// #define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
// #define HW_ICU_DEV				ICUD4
// #define HW_ICU_CHANNEL			ICU_CHANNEL_1
// #define HW_ICU_GPIO_AF			GPIO_AF_TIM4
// #define HW_ICU_GPIO				GPIOB
// #define HW_ICU_PIN				6

// --- JAH added KILL I2C TO REUSE PINS, POISON THE DEFINES AT THE BOTTOM OF THIS FILE.
// I2C Peripheral
// #define HW_I2C_DEV				I2CD2
// #define HW_I2C_GPIO_AF			GPIO_AF_I2C2
// #define HW_I2C_SCL_PORT			GPIOB
// #define HW_I2C_SCL_PIN			10
// #define HW_I2C_SDA_PORT			GPIOB
// #define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// --- JAH added KILL SPI1 TO REUSE PINS, POISON THE DEFINES AT THE BOTTOM OF THIS FILE.
// SPI pins
// #define HW_SPI_DEV				SPID1
// #define HW_SPI_GPIO_AF			GPIO_AF_SPI1
// #define HW_SPI_PORT_NSS			GPIOB
// #define HW_SPI_PIN_NSS			11
// //#define HW_SPI_PORT_SCK			GPIOA
// //#define HW_SPI_PIN_SCK			5
// #define HW_SPI_PORT_MOSI		GPIOB
// #define HW_SPI_PIN_MOSI			10
// #define HW_SPI_PORT_MISO		GPIOA
// #define HW_SPI_PIN_MISO			6

// IMU
#define IMU_DEV				IMU_DEV_LSM6DS3
#define IMU_COM				IMU_COM_SPI_HW
#define IMU_SPI_DEV			SPID3
#define IMU_SPI_AF			GPIO_AF_SPI3
#define IMU_SPI_NSS_GPIO		GPIOA
#define IMU_SPI_NSS_PIN			15
#define IMU_SPI_SCK_GPIO		GPIOB
#define IMU_SPI_SCK_PIN			3
#define IMU_SPI_MOSI_GPIO		GPIOB
#define IMU_SPI_MOSI_PIN		5
#define IMU_SPI_MISO_GPIO		GPIOB
#define IMU_SPI_MISO_PIN		4
#define IMU_FLIP

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

#define HW_DEAD_TIME_NSEC		300.0

// Default setting overrides

#ifdef MCCONF_L_MIN_VOLTAGE
#undef MCCONF_L_MIN_VOLTAGE
#endif
#define MCCONF_L_MIN_VOLTAGE			8.0		// Minimum input voltage

#ifdef MCCONF_L_MAX_VOLTAGE
#undef MCCONF_L_MAX_VOLTAGE
#endif
#define MCCONF_L_MAX_VOLTAGE			94.0	// Maximum input voltage

#ifdef MCCONF_L_CURRENT_MAX
#undef MCCONF_L_CURRENT_MAX
#endif
#define MCCONF_L_CURRENT_MAX			300.0    // Current limit in Amperes (Upper)

#ifdef MCCONF_L_CURRENT_MIN
#undef MCCONF_L_CURRENT_MIN
#endif
#define MCCONF_L_CURRENT_MIN			-200.0	// Current limit in Amperes (Lower)

#ifdef MCCONF_FOC_F_ZV
#undef MCCONF_FOC_F_ZV
#endif
#define MCCONF_FOC_F_ZV					30000.0

#ifdef MCCONF_L_MAX_ABS_CURRENT
#undef MCCONF_L_MAX_ABS_CURRENT
#endif
#define MCCONF_L_MAX_ABS_CURRENT		400.0	// The maximum absolute current above which a fault is generated

#ifdef MCCONF_FOC_SAMPLE_V0_V7
#undef MCCONF_FOC_SAMPLE_V0_V7
#endif
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)

#ifdef MCCONF_L_IN_CURRENT_MAX	
#undef MCCONF_L_IN_CURRENT_MAX	
#endif
#define MCCONF_L_IN_CURRENT_MAX			300.0	// Input current limit in Amperes (Upper)

#ifdef MCCONF_L_IN_CURRENT_MIN	
#undef MCCONF_L_IN_CURRENT_MIN	
#endif
#define MCCONF_L_IN_CURRENT_MIN			-200.0	// Input current limit in Amperes (Lower)

#ifdef APPCONF_APP_TO_USE
#undef APPCONF_APP_TO_USE
#endif
#define APPCONF_APP_TO_USE				APP_NONE

#ifdef MCCONF_L_RPM_MAX
#undef MCCONF_L_RPM_MAX
#endif
#define MCCONF_L_RPM_MAX				160000.0	// The motor speed limit (Upper)ERPM


// Setting limits
#define HW_LIM_CURRENT			-410.0, 410.0
#define HW_LIM_CURRENT_IN		-410.0, 410.0
#define HW_LIM_CURRENT_ABS		0.0, 600.0
#define HW_LIM_VIN				8.0, 97.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 1.0
#define HW_LIM_TEMP_FET			-40.0, 110.0

// JAH ADDED SETTING OVERRIDES ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ============================================================================
// --- WRSM MOTOR-GENERATOR INDUCTOR SAFETY OVERRIDES ---
// ============================================================================
// Force-disables low-side MOSFET shorting at zero duty / standstill.
// Shorting phases on a WRSM can generate violent dynamic locking torque at speed
// and causes the controller to trigger self-latching false ESTOPs on stop.
#define JAH_HW_DISABLE_LOW_SIDE_SHORT_ON_ZERO_DUTY
#ifdef MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY
#undef MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY
#endif
#define MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY false

// Force default observer type to MXLEMMING_LAMBDA_COMP
#ifdef MCCONF_FOC_OBSERVER_TYPE
#undef MCCONF_FOC_OBSERVER_TYPE
#endif
#define MCCONF_FOC_OBSERVER_TYPE        3 // 3 = FOC_OBSERVER_MXLEMMING_LAMBDA_COMP

#ifdef MCCONF_FOC_OBSERVER_OFFSET
#undef MCCONF_FOC_OBSERVER_OFFSET
#endif
#define MCCONF_FOC_OBSERVER_OFFSET		0.15 // default is -1.0. 

#ifdef MCCONF_FOC_OBSERVER_GAIN_SLOW
#undef MCCONF_FOC_OBSERVER_GAIN_SLOW
#endif
#define MCCONF_FOC_OBSERVER_GAIN_SLOW	0.05

// Set saturation compensation off.
#ifdef MCCONF_FOC_SAT_COMP_MODE
#undef MCCONF_FOC_SAT_COMP_MODE
#endif
#define MCCONF_FOC_SAT_COMP_MODE        0 // 0 = SAT_COMP_DISABLED

// Force MTPA Mode to 'IQ Target' by default
#ifdef MCCONF_FOC_MTPA_MODE
#undef MCCONF_FOC_MTPA_MODE
#endif
#define MCCONF_FOC_MTPA_MODE            0 // 0 = MTPA_MODE_OFF 1 = MTPA_MODE_IQ_TARGET

// Force full Speed-compensated Cross-coupling & BEMF Decoupling by default
#ifdef MCCONF_FOC_CC_DECOUPLING
#undef MCCONF_FOC_CC_DECOUPLING
#endif
#define MCCONF_FOC_CC_DECOUPLING        3 // 3 = FOC_CC_DECOUPLING_CROSS_BEMF

// Force default motor R value.
#ifdef MCCONF_FOC_MOTOR_R
#undef MCCONF_FOC_MOTOR_R
#endif
#define MCCONF_FOC_MOTOR_R				0.01160 //average stator resistance at 22 °C

// Force default number of poles to 16 (8 pole pairs)
#ifdef MCCONF_FOC_NO_POLES
#undef MCCONF_FOC_NO_POLES
#endif
#define MCCONF_FOC_NO_POLES             16

#define POLE_PAIRS ((float)MCCONF_FOC_NO_POLES / 2.0f)

// --- FIELD RESISTANCE
#define MGU_FIELD_R                  5.60    //Ohm field resistance at 22 °C
#define MGU_FIELD_L                  1.912   //H 1912mH determined by telemetry analysis 20260821 JAH 
// --- FIELD CURRENT SENSOR (ACS712-05B STANDARD DIRECT VIA ONBOARD 10k/10k) ---
#define FIELD_CURRENT_SENSOR_UNI_DIRECTIONAL  1       // 1 for forward-only tracking
#define FIELD_CURRENT_VOLTAGE_OFFSET_V        1.234f  // Your physical calibrated 0.0A rest voltage on the pin

// Attenuated scale: +185mV/A standard sensitivity * 0.4936 actual divider factor = +91.316 mV/A
#define FIELD_CURRENT_SENSOR_VOLTS_PER_AMP    0.091316f  
#define FIELD_CURRENT_SENSOR_AMPS_PER_VOLT    (1.0f / FIELD_CURRENT_SENSOR_VOLTS_PER_AMP)

// --- DYNAMIC PARAMETRIC FAULT MONITORING BOUNDARIES ---
// Normal operational envelope tracks from 1.234V (0A) up to 1.501V (2.92A)
// If the wire snaps or chip loses 5V power, the onboard pull-down drags the line to 0.0V
#define FIELD_CURRENT_FAULT_VOLTAGE_MIN       0.80f   // Catches broken wire, lost 5V, or dead sensor chip
#define FIELD_CURRENT_FAULT_VOLTAGE_MAX       2.20f   // Catches raw sensor rail overvoltage surges
#define FIELD_CURRENT_FAULT_DEBOUNCE_CYCLES   100     // 5ms filter debounce window

// --- JAH: WRSM Motor-Generator Unit Cranking Defaults ---
#define MCCONF_WRSM_CRANK_TARGET_RPM          6000.0f  // 600 engine RPM * 8 PP
#define MCCONF_WRSM_CRANK_RAMP_TIME           5.0f    // 10s starting ramp time
#define MCCONF_WRSM_CRANK_TARGET_IQ           200.0f   // 100A starting current target
// Force Open-Loop Handoff Speed to exactly 1000.0 ERPM
#ifdef MCCONF_FOC_OPENLOOP_RPM
#undef MCCONF_FOC_OPENLOOP_RPM
#endif
#define MCCONF_FOC_OPENLOOP_RPM         1000.0f

#ifdef MCCONF_S_PID_SPEED_SOURCE
#undef MCCONF_S_PID_SPEED_SOURCE
#endif
//#define MCCONF_S_PID_SPEED_SOURCE		S_PID_SPEED_SRC_PLL //default, has as much as 20ms delay.
#define MCCONF_S_PID_SPEED_SOURCE    S_PID_SPEED_SRC_FASTER //estimate 1ms delay. may be too noisy.
// can try S_PID_SPEED_SRC_PLL but change UTILS_LP_FAST(motor_now->m_speed_est_fast, diff * fs, 0.01); to
// UTILS_LP_FAST(motor_now->m_speed_est_fast, diff * fs, 0.05); 20ms for 0.01 to 4ms for 0.05.

// --- JAH: WRSM Motor-Generator Unit Alternator Defaults ---
#define MCCONF_WRSM_ALT_TARGET_VOLTAGE        14.2f    // Regulate 12V bus to 14.2V
#define MCCONF_WRSM_ALT_BATT_CHARGE_LIMIT     20.0f    // Clamp direct battery charge to 20A max
#define MCCONF_WRSM_ALT_MAX_IQ                250.0f   // Let stator generate up to 250A for loads
#define MCCONF_WRSM_ALT_CAN_TIMEOUT_MS        250.0f   // Fall back to voltage mode after 250ms silence

// --- JAH: WRSM Motor-Generator Unit Stall Catch Defaults ---
#define MCCONF_WRSM_STALL_CATCH_TRIGGER_RPM   1200.0f  // Catch engine if sags below 150 MGU RPM (18.75 Hz)
#define MCCONF_WRSM_STALL_CATCH_TARGET_RPM    2400.0f  // Motoring target speed for catch mode (300 ERPM)
#define MCCONF_WRSM_STALL_CATCH_MAX_IQ        150.0f   // Deliver up to 150A to catch the block
#define MCCONF_WRSM_STALL_CATCH_KP            0.15f    // Stiff proportional gain for transient catches
#define MCCONF_WRSM_STALL_CATCH_KI            0.080f   // Stiff integral gain to settle caught idle quickly
#define MCCONF_WRSM_STALL_DECEL_TRIGGER       -800.0f  // Catch engine if deceleration exceeds -800 ERPM/s^2
#define MCCONF_WRSM_ACCEL_FILTER_COEF         0.05f    // Clean low-pass filter coefficient for d_speed/dt

// Speed control PID settings
// needs to work down to zero rpm.
#ifdef MCCONF_S_PID_MIN_ERPM
#undef MCCONF_S_PID_MIN_ERPM
#endif
#define MCCONF_S_PID_MIN_RPM           0.0f
// no braking so that we dont try to brake the engine as it fires.
#ifdef MCCONF_S_PID_ALLOW_BRAKING
#undef MCCONF_S_PID_ALLOW_BRAKING
#endif
#define MCCONF_S_PID_ALLOW_BRAKING      false

// 60A instant open-loop starting boost current for engine cranking
#ifdef MCCONF_FOC_SL_OPENLOOP_BOOST_Q
#undef MCCONF_FOC_SL_OPENLOOP_BOOST_Q
#endif
#define MCCONF_FOC_SL_OPENLOOP_BOOST_Q  0.0f

// Force the open-loop scaling ratio to 1.0. 
// (This guarantees the handoff target stays locked at exactly 1000 ERPM 
// instead of letting VESC scale it down under heavy cranking currents!)
#ifdef MCCONF_FOC_OPENLOOP_RPM_LOW
#undef MCCONF_FOC_OPENLOOP_RPM_LOW
#endif
#define MCCONF_FOC_OPENLOOP_RPM_LOW     1.0f

// FOR USE WITH ENCODER STARTS.. ERPM to hand off to sensorless observer.
//#ifdef MCCONF_FOC_SL_ERPM
//#undef MCCONF_FOC_SL_ERPM
//#endif
//#define MCCONF_FOC_SL_ERPM        1000.0f

// FOR USE WITH ENCODER STARTS... ERPM to start blending with sensorless observer.
//#ifdef MCCONF_FOC_SL_ERPM_START
//#undef MCCONF_FOC_SL_ERPM_START
//#endif
//#define MCCONF_FOC_SL_ERPM_START  800.0f

// Open-Loop Ramp Timing
// Time before we start spinning
#ifdef MCCONF_FOC_SL_OPENLOOP_T_LOCK
#undef MCCONF_FOC_SL_OPENLOOP_T_LOCK
#endif
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK  0.0f
// Time to reach MCCONF_FOC_OPENLOOP_RPM, DEFAULT VALUE, can be edited!
#ifdef MCCONF_FOC_SL_OPENLOOP_T_RAMP
#undef MCCONF_FOC_SL_OPENLOOP_T_RAMP
#endif
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP  1.0f
// Time to hold at MCCONF_FOC_SL_ERPM before switching to sensorless control
#ifdef MCCONF_FOC_SL_OPENLOOP_TIME
#undef MCCONF_FOC_SL_OPENLOOP_TIME
#endif
#define MCCONF_FOC_SL_OPENLOOP_TIME    0.00f

// Tuned Speed PID gains for 300A Max Automotive Engine Cranking
// THESE WILL BE CONFIG DEFAULT VALUES.
// stock P is 0.002, recommended was 0.015
#ifdef MCCONF_S_PID_KP
#undef MCCONF_S_PID_KP
#endif
#define MCCONF_S_PID_KP                0.10f
// P_term_normalized = Speed_Error * s_pid_kp * (1.0 / 20.0)
// Iq_set_proportional = P_term_normalized * l_current_max
// With current limit set to 300.0 A, the equation simplifies to:
// Iq_set_proportional = Speed_Error * s_pid_kp * 0.05 * 300
// Iq_set_proportional = Speed_Error * s_pid_kp * 15

#ifdef MCCONF_S_PID_KI
#undef MCCONF_S_PID_KI
#endif
#define MCCONF_S_PID_KI                0.040f

#ifdef MCCONF_S_PID_KD
#undef MCCONF_S_PID_KD
#endif
#define MCCONF_S_PID_KD                0.0003f

#ifdef MCCONF_S_PID_KD_FILTER
#undef MCCONF_S_PID_KD_FILTER
#endif
#define MCCONF_S_PID_KD_FILTER         0.20f

// Disable app control signal timeout (ignore missing RC control inputs)
#ifdef APPCONF_TIMEOUT_MSEC
#undef APPCONF_TIMEOUT_MSEC
#endif
#define APPCONF_TIMEOUT_MSEC           0


// JAH ADDED FAST LOOKUP TABLES ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MGU_LOOKUP_SECTORS 5

typedef struct {
    float lower_bound_if;
    
    // --- BASE VECTOR ATTRIBUTES (SI UNITS) ---
    float base_flux;
    float base_ld;
    float base_lq;
    float base_l;
    float base_ld_lq_diff;
    float base_inv_ld;
    float base_inv_lq;
    //float base_p_inv_ld_lq;
    
    // --- PRECOMPUTED 1-CYCLE MULTIPLY SLOPES ---
    float slope_flux;
    float slope_ld;
    float slope_lq;
    float slope_l;
    float slope_ld_lq_diff;
    float slope_inv_ld;
    float slope_inv_lq;
    //float slope_p_inv_ld_lq;
} if_lookup_row_t;

static const if_lookup_row_t mgu_if_table[MGU_LOOKUP_SECTORS] = {
    // --- ROW 0: OVERCURRENT CEILING TRACKING (& ANCHOR LIMIT >= 2.92A) ---
    {
        .lower_bound_if = 2.92f,
        .base_flux = 0.01320000f, .base_ld = 0.0000140900f, .base_lq = 0.0000243500f, .base_l = 0.0000192200f, .base_ld_lq_diff = 0.0000102600f,
        .base_inv_ld = 70972.3203125f, .base_inv_lq = 41067.7617188f, 
        .slope_flux = 0.00000001f, .slope_ld = 0.00000000f, .slope_lq = 0.00000000f, .slope_l = 0.00000000f, .slope_ld_lq_diff = 0.00000000f,
        .slope_inv_ld = 0.00000000f, .slope_inv_lq = 0.00000000f
    },
    // --- ROW 1: INTERVAL SECTOR 2.00A TO 2.92A ---
    {
        .lower_bound_if = 2.00f,
        .base_flux = 0.01241000f, .base_ld = 0.0000160900f, .base_lq = 0.0000281500f, .base_l = 0.0000221200f, .base_ld_lq_diff = 0.0000120600f,
        .base_inv_ld = 62150.4023438f, .base_inv_lq = 35523.9765625f, 
        .slope_flux = 0.00085870f, .slope_ld = -0.0000021739f, .slope_lq = -0.0000041304f, .slope_l = -0.0000031522f, .slope_ld_lq_diff = -0.0000019565f,
        .slope_inv_ld = 9589.0410156f, .slope_inv_lq = 6025.8535156f, 
    },
    // --- ROW 2: INTERVAL SECTOR 1.00A TO 2.00A ---
    {
        .lower_bound_if = 1.00f,
        .base_flux = 0.01008000f, .base_ld = 0.0000273400f, .base_lq = 0.0000431600f, .base_l = 0.0000352500f, .base_ld_lq_diff = 0.0000158200f,
        .base_inv_ld = 36576.4453125f, .base_inv_lq = 23169.6015625f, 
        .slope_flux = 0.00233000f, .slope_ld = -0.0000112500f, .slope_lq = -0.0000150100f, .slope_l = -0.0000131300f, .slope_ld_lq_diff = -0.0000037600f,
        .slope_inv_ld = 25573.9570312f, .slope_inv_lq = 12354.3750000f, 
    },
    // --- ROW 3: INTERVAL SECTOR 0.50A TO 1.00A ---
    {
        .lower_bound_if = 0.50f,
        .base_flux = 0.00680000f, .base_ld = 0.0000298550f, .base_lq = 0.0000461250f, .base_l = 0.0000379900f, .base_ld_lq_diff = 0.0000162700f,
        .base_inv_ld = 33495.2265625f, .base_inv_lq = 21680.2167969f, 
        .slope_flux = 0.00656000f, .slope_ld = -0.0000050300f, .slope_lq = -0.0000059300f, .slope_l = -0.0000054800f, .slope_ld_lq_diff = -0.0000009000f,
        .slope_inv_ld = 6162.4370117f, .slope_inv_lq = 2978.7695312f, 
    },
    // --- ROW 4: FLOOR CEILING TRANSITION (0.00A TO 0.50A & NEGATIVE CURRENT BOUNDARY PROTECTION) ---
    {
        .lower_bound_if = 0.00f,
        .base_flux = 0.00308000f, .base_ld = 0.0000301300f, .base_lq = 0.0000460700f, .base_l = 0.0000381000f, .base_ld_lq_diff = 0.0000159400f,
        .base_inv_ld = 33189.5117188f, .base_inv_lq = 21706.0996094f, 
        .slope_flux = 0.00744000f, .slope_ld = -0.0000005500f, .slope_lq = 0.0000001100f, .slope_l = -0.0000002200f, .slope_ld_lq_diff = 0.0000006600f,
        .slope_inv_ld = 611.4300537f, .slope_inv_lq = -51.7656250f, 
    }
};

// Functions
void smart_switch_thread_start(void);
void smart_switch_pin_init(void);
bool smart_switch_is_pressed(void);
void smart_switch_shut_down(void);
void smart_switch_keep_on(void);
float hw_classicp_get_temp(void);


// ============================================================================
// --- JAH added WRSM FORK POISONING SENTINELS FOR COMM PORT RECLAIM
// Prevents compile if any file attempts to start or use USART3 / SD3
// ============================================================================
#ifdef HW_UART_DEV
#undef HW_UART_DEV
#endif
#define HW_UART_DEV             BLOCKED_USART3_DEV_ERROR

#ifdef SD3
#undef SD3
#endif
#define SD3                     BLOCKED_USART3_SD3_ERROR

#ifdef HW_UART_TX_PORT
#undef HW_UART_TX_PORT
#endif
#define HW_UART_TX_PORT         BLOCKED_GPIOB10_TX_PORT_ERROR

#ifdef HW_UART_RX_PORT
#undef HW_UART_RX_PORT
#endif
#define HW_UART_RX_PORT         BLOCKED_GPIOB11_RX_PORT_ERROR

#ifdef HW_UART_TX_PIN
#undef HW_UART_TX_PIN
#endif
#define HW_UART_TX_PIN         BLOCKED_GPIOB10_TX_PIN_ERROR

#ifdef HW_UART_RX_PIN
#undef HW_UART_RX_PIN
#endif
#define HW_UART_RX_PIN         BLOCKED_GPIOB11_RX_PIN_ERROR
#define HW_UART_DEV_BLOCKED

// ============================================================================
// --- JAH added WRSM FORK POISONING SENTINELS FOR PPM / SERVO TIM4 RECLAIM
// Prevents compile if any file attempts to configure or use TIM4 for servo/ppm input
// ============================================================================
#ifdef HW_USE_SERVO_TIM4
#undef HW_USE_SERVO_TIM4
#endif
#define HW_USE_SERVO_TIM4             BLOCKED_TIM4_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_TIMER
#undef HW_ICU_TIMER
#endif
#define HW_ICU_TIMER                  BLOCKED_TIM4_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_TIM_CLK_EN
#undef HW_ICU_TIM_CLK_EN
#endif
#define HW_ICU_TIM_CLK_EN()           BLOCKED_TIM4_REPURPOSED_FOR_WRSM_FIELD_PWM_CLK

#ifdef HW_ICU_DEV
#undef HW_ICU_DEV
#endif
#define HW_ICU_DEV                    BLOCKED_ICUD4_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_CHANNEL
#undef HW_ICU_CHANNEL
#endif
#define HW_ICU_CHANNEL                BLOCKED_ICU_CHANNEL_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_GPIO_AF
#undef HW_ICU_GPIO_AF
#endif
#define HW_ICU_GPIO_AF                BLOCKED_TIM4_AF_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_GPIO
#undef HW_ICU_GPIO
#endif
#define HW_ICU_GPIO                   BLOCKED_GPIOB_6_REPURPOSED_FOR_WRSM_FIELD_PWM

#ifdef HW_ICU_PIN
#undef HW_ICU_PIN
#endif
#define HW_ICU_PIN                    BLOCKED_PIN_6_REPURPOSED_FOR_WRSM_FIELD_PWM
#define HW_ICU_GPIO_BLOCKED

// ============================================================================
// --- JAH added WRSM FORK POISONING SENTINELS FOR I2C2 PORT RECLAIM
// Prevents compile if any background app tries to use I2C2 on GPIOB_10 / GPIOB_11
// ============================================================================
#ifdef HW_I2C_DEV
#undef HW_I2C_DEV
#endif
#define HW_I2C_DEV              BLOCKED_I2C2_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_I2C_GPIO_AF
#undef HW_I2C_GPIO_AF
#endif
#define HW_I2C_GPIO_AF          BLOCKED_I2C2_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_I2C_SCL_PORT
#undef HW_I2C_SCL_PORT
#endif
#define HW_I2C_SCL_PORT         BLOCKED_GPIOB10_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_I2C_SCL_PIN
#undef HW_I2C_SCL_PIN
#endif
#define HW_I2C_SCL_PIN          BLOCKED_PIN_10_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_I2C_SDA_PORT
#undef HW_I2C_SDA_PORT
#endif
#define HW_I2C_SDA_PORT         BLOCKED_GPIOB11_REPURPOSED_FOR_WRSM_RX_GPIO

#ifdef HW_I2C_SDA_PIN
#undef HW_I2C_SDA_PIN
#endif
#define HW_I2C_SDA_PIN          BLOCKED_PIN_11_REPURPOSED_FOR_WRSM_RX_GPIO
#define HW_I2C_DEV_BLOCKED

// ============================================================================
// --- JAH added WRSM FORK POISONING SENTINELS FOR SPI1 PORT RECLAIM
// Prevents compile if any background app tries to use SPI1 on GPIOB_10 / GPIOB_11
// ============================================================================
#ifdef HW_SPI_DEV
#undef HW_SPI_DEV
#endif
#define HW_SPI_DEV              BLOCKED_SPI1_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_SPI_GPIO_AF
#undef HW_SPI_GPIO_AF
#endif
#define HW_SPI_GPIO_AF          BLOCKED_SPI1_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_SPI_PORT_NSS
#undef HW_SPI_PORT_NSS
#endif
#define HW_SPI_PORT_NSS         BLOCKED_GPIOB11_REPURPOSED_FOR_WRSM_RX_GPIO

#ifdef HW_SPI_PIN_NSS
#undef HW_SPI_PIN_NSS
#endif
#define HW_SPI_PIN_NSS          BLOCKED_PIN_11_REPURPOSED_FOR_WRSM_RX_GPIO

#ifdef HW_SPI_PORT_MOSI
#undef HW_SPI_PORT_MOSI
#endif
#define HW_SPI_PORT_MOSI        BLOCKED_GPIOB10_REPURPOSED_FOR_WRSM_FIELD_EN

#ifdef HW_SPI_PIN_MOSI
#undef HW_SPI_PIN_MOSI
#endif
#define HW_SPI_PIN_MOSI         BLOCKED_PIN_10_REPURPOSED_FOR_WRSM_FIELD_EN
#define HW_SPI_DEV_BLOCKED
// ============================================================================

#endif /* HW_CLASSICP_CORE_H_ */

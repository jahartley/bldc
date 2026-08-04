/*
 * field_driver.c
 *
 * Driver for controlling rotor field coil via BTS7960 H-Bridge.
 * Uses TIM4 CH1 (PB6) for 5 kHz PWM output and PA6 (AD2 / COMM Header) for Enable.
 */

#include "field_driver.h"
#include "hw.h"
#include "commands.h"
#include "mc_interface.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "pwm_servo.h"
#include "utils_math.h"

// 5 kHz PWM setup parameters
#define FIELD_PWM_FREQ         5000
#define TIM4_CLOCK_FREQ        (SYSTEM_CORE_CLOCK / 2) // APB1 Timer clock = 84 MHz
#define FIELD_PWM_PERIOD       (TIM4_CLOCK_FREQ / FIELD_PWM_FREQ) // 16800 counts for 5 kHz

static volatile float current_duty = 0.0f;
static volatile bool is_enabled = false;
static volatile float zero_offset_volts = 1.667f; // Default ~2.5V / 1.5
static volatile bool acs712_fault = false;

// ACS712-05B Constants (5A Version)
#define ACS712_SENSITIVITY      0.185f  // V/A (185 mV/A)
#define ACS712_DIVIDER_RATIO    3.00f   // (20k series + 10k GND) / 10k
#define ACS712_MIN_VALID_VOLTS  0.10f   // Voltage threshold for sensor disconnect fault

// Detection Data Points (5 points from detection CSV)
#define LOOKUP_POINTS 5
static const float lookup_ifield[LOOKUP_POINTS]    = {0.00f,      0.50f,      1.00f,      2.00f,      2.92f};
static const float lookup_ls[LOOKUP_POINTS]        = {38.10e-6f,  37.99e-6f,  35.25e-6f,  22.12e-6f,  19.22e-6f};
static const float lookup_ld[LOOKUP_POINTS]        = {30.13e-6f,  29.86e-6f,  27.34e-6f,  16.09e-6f,  14.09e-6f};
static const float lookup_lq[LOOKUP_POINTS]        = {46.07e-6f,  46.13e-6f,  43.16e-6f,  28.15e-6f,  24.35e-6f};
static const float lookup_diff[LOOKUP_POINTS]      = {15.94e-6f,  16.27e-6f,  15.82e-6f,  12.06e-6f,  10.26e-6f};
static const float lookup_flux[LOOKUP_POINTS]      = {0.00308f,   0.00680f,   0.01008f,   0.01241f,   0.01320f};

void field_driver_calibrate_zero(void) {
	// Clear any latched fault to allow calibration
	acs712_fault = false;

	// 1. Set Enable HIGH and PWM to 0.0%
	bool prev_enable = is_enabled;
	palSetPad(GPIOB, 10);
	TIM4->CCR1 = 0;

	// 2. Wait 200 ms for field winding inductive current decay
	chThdSleepMilliseconds(200);

	// 3. Average 100 ADC readings on ADC_EXT (PA7 / ADC_IND_EXT)
	float sum_volts = 0.0f;
	for (int i = 0; i < 100; i++) {
		sum_volts += ADC_VOLTS(ADC_IND_EXT);
		chThdSleepMilliseconds(1);
	}
	zero_offset_volts = sum_volts / 100.0f;

	// 4. Restore state
	if (is_enabled) {
		palSetPad(GPIOB, 10);
	} else {
		palClearPad(GPIOB, 10);
	}
	TIM4->CCR1 = (uint32_t)(current_duty * (float)FIELD_PWM_PERIOD);
}

float field_driver_get_raw_adc_volts(void) {
	return ADC_VOLTS(ADC_IND_EXT);
}

float field_driver_get_zero_offset_volts(void) {
	return zero_offset_volts;
}

bool field_driver_has_fault(void) {
	return acs712_fault;
}

void field_driver_clear_fault(void) {
	acs712_fault = false;
}

float field_driver_get_current(void) {
	float raw_v = ADC_VOLTS(ADC_IND_EXT);

	// Check for ACS712 disconnect / low-voltage fault (< 0.10V)
	if (raw_v < ACS712_MIN_VALID_VOLTS) {
		if (!acs712_fault) {
			commands_printf("FAULT DETECTED: ACS712 sensor disconnected or voltage < 0.10V (Raw: %.3f V)\n", (double)raw_v);
		}
		acs712_fault = true;
		return 0.0f; // Return 0.0A so lookups safely map to 0A defaults
	}

	if (acs712_fault) {
		// Locked out while fault persists
		return 0.0f;
	}

	float delta_v = (raw_v - zero_offset_volts) * ACS712_DIVIDER_RATIO;
	float i_field = delta_v / ACS712_SENSITIVITY;
	if (i_field < 0.0f) {
		i_field = 0.0f;
	}
	return i_field;
}

// Helper piecewise linear interpolation function
static float interpolate_table(float i_field, const float *tbl) {
	if (i_field <= lookup_ifield[0]) {
		return tbl[0];
	}
	if (i_field >= lookup_ifield[LOOKUP_POINTS - 1]) {
		return tbl[LOOKUP_POINTS - 1];
	}

	for (int i = 0; i < LOOKUP_POINTS - 1; i++) {
		if (i_field >= lookup_ifield[i] && i_field <= lookup_ifield[i + 1]) {
			float t = (i_field - lookup_ifield[i]) / (lookup_ifield[i + 1] - lookup_ifield[i]);
			return tbl[i] + t * (tbl[i + 1] - tbl[i]);
		}
	}

	return tbl[0];
}

float field_get_foc_l(void) {
	return interpolate_table(field_driver_get_current(), lookup_ls);
}

float field_get_foc_ld(void) {
	return interpolate_table(field_driver_get_current(), lookup_ld);
}

float field_get_foc_lq(void) {
	return interpolate_table(field_driver_get_current(), lookup_lq);
}

float field_get_foc_ld_lq_diff(void) {
	return interpolate_table(field_driver_get_current(), lookup_diff);
}

float field_get_foc_flux_linkage(void) {
	return interpolate_table(field_driver_get_current(), lookup_flux);
}

void field_driver_init(void) {
	// 1. Ensure servo decoder and PWM servo on TIM4 are stopped
	if (servodec_is_running()) {
		servodec_stop();
	}
	pwm_servo_stop();

	// 2. Enable GPIO and Timer clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// 3. Configure PB10 (TX Pin / Enable Pin) as Digital Output
	palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	field_driver_set_enable(false);

	// 4. Configure PB6 (PWM Pin) as Alternate Function TIM4_CH1
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(GPIO_AF_TIM4) | PAL_STM32_OSPEED_HIGHEST);

	// 5. Setup TIM4 Time Base for 5 kHz PWM
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = FIELD_PWM_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// 6. Setup Output Compare Channel 1 for PWM Mode 1
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	// 7. Enable Counter
	TIM_Cmd(TIM4, ENABLE);

	field_driver_set_duty(0.0f);

	// 8. Calibrate ACS712 zero offset at startup
	field_driver_calibrate_zero();

	field_driver_conf_init();
}

void field_driver_set_enable(bool enable) {
	if (acs712_fault && enable) {
		commands_printf("FIELD FAULT: Enable blocked because ACS712 sensor is disconnected or low voltage!\n");
		return;
	}
	is_enabled = enable;
	
	// Enforce GPIO output push-pull mode whenever state changes
	palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	
	if (enable) {
		palSetPad(GPIOB, 10);
	} else {
		palClearPad(GPIOB, 10);
	}
}

bool field_driver_get_enable(void) {
	return is_enabled;
}

void field_driver_set_duty(float duty) {
	if (acs712_fault) {
		current_duty = 0.0f;
		TIM4->CCR1 = 0;
		return;
	}
	utils_truncate_number(&duty, 0.0f, 1.0f);
	current_duty = duty;

	uint32_t compare_value = (uint32_t)(duty * (float)FIELD_PWM_PERIOD);
	TIM4->CCR1 = compare_value;
}

float field_driver_get_duty(void) {
	return current_duty;
}

// Custom Config XML definition for VESC Tool UI
static const char field_config_xml[] =
		"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
		"<vconfig name=\"field_config\">\n"
		"    <category name=\"Field Coil Control\">\n"
		"        <param name=\"field_enable\" type=\"bool\" label=\"Enable Field Coil Driver\">\n"
		"            <help>Enable or disable BTS7960 H-Bridge driver via PC14 GPIO.</help>\n"
		"        </param>\n"
		"        <param name=\"field_duty\" type=\"double\" label=\"Field Duty Cycle\" min=\"0.0\" max=\"1.0\" step=\"0.01\" suffix=\"%\">\n"
		"            <help>Rotor field coil PWM duty cycle (0.0 to 1.0).</help>\n"
		"        </param>\n"
		"        <param name=\"field_current\" type=\"double\" label=\"Measured Field Current\" min=\"0.0\" max=\"10.0\" step=\"0.01\" suffix=\"A\">\n"
		"            <help>Measured ACS712 field current in Amperes.</help>\n"
		"        </param>\n"
		"    </category>\n"
		"</vconfig>\n";

#include "conf_custom.h"
#include "buffer.h"

int field_driver_get_cfg(uint8_t *buffer, bool is_default) {
	int32_t ind = 0;
	if (is_default) {
		buffer[ind++] = 0; // disabled by default
		buffer_append_float32_auto(buffer, 0.0f, &ind);
		buffer_append_float32_auto(buffer, 0.0f, &ind);
	} else {
		buffer[ind++] = is_enabled ? 1 : 0;
		buffer_append_float32_auto(buffer, current_duty, &ind);
		buffer_append_float32_auto(buffer, field_driver_get_current(), &ind);
	}
	return ind;
}

bool field_driver_set_cfg(uint8_t *buffer) {
	int32_t ind = 0;
	bool enable = buffer[ind++] != 0;
	float duty = buffer_get_float32_auto(buffer, &ind);

	field_driver_set_enable(enable);
	field_driver_set_duty(duty);
	return true;
}

int field_driver_get_cfg_xml(uint8_t **data) {
	*data = (uint8_t*)field_config_xml;
	return sizeof(field_config_xml);
}

void field_driver_conf_init(void) {
	conf_custom_add_config(field_driver_get_cfg, field_driver_set_cfg, field_driver_get_cfg_xml);
}

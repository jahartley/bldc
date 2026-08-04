/*
 * field_driver.h
 *
 * Driver for controlling rotor field coil via BTS7960 H-Bridge.
 * Uses TIM4 CH1 (PB6) for 5 kHz PWM output and PC14 (AUX_GPIO) for Enable.
 */

#ifndef FIELD_DRIVER_H_
#define FIELD_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

void field_driver_init(void);
void field_driver_set_enable(bool enable);
bool field_driver_get_enable(void);
void field_driver_set_duty(float duty);
float field_driver_get_duty(void);

// ACS712 Current Sensing & Auto-Calibration
void field_driver_calibrate_zero(void);
float field_driver_get_current(void);
float field_driver_get_raw_adc_volts(void);
float field_driver_get_zero_offset_volts(void);
bool field_driver_has_fault(void);
void field_driver_clear_fault(void);

// Dynamic FOC Parameter Lookup Functions (based on measured field current)
float field_get_foc_l(void);          // Average Inductance Ls in Henries
float field_get_foc_ld(void);         // Ld in Henries
float field_get_foc_lq(void);         // Lq in Henries
float field_get_foc_ld_lq_diff(void); // (Lq - Ld) in Henries
float field_get_foc_flux_linkage(void);

// Custom XML & Configuration Interface for VESC Tool
void field_driver_conf_init(void);
int field_driver_get_cfg(uint8_t *buffer, bool is_default);
bool field_driver_set_cfg(uint8_t *buffer);
int field_driver_get_cfg_xml(uint8_t **data);

#endif /* FIELD_DRIVER_H_ */

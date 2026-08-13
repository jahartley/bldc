#ifndef WRSM_FIELD_CONTROLLER_H_
#define WRSM_FIELD_CONTROLLER_H_

#include "foc_math.h"

// ============================================================================
// --- JAH: WRSM ROTOR FIELD REGULATION INTERFACE ---
// ============================================================================


#ifndef WRSM_FIELD_DISABLE
#define WRSM_FIELD_DISABLE() palClearPad(HW_FIELD_EN_GPIO, HW_FIELD_EN_PIN)
#endif

// --- Level 1: Static Inline High-Speed parameter injection (Executed in FOC ISR) ---
/**
 * @brief  High-speed, saturation-aware motor parameter injection.
 * @note   Executed inside the ultra-critical 15 kHz FOC ADC DMA interrupt context.
 *         Must be compiled as static inline to eliminate stack frame overhead.
 * 
 * This function performs a division-free, piecewise linear interpolation using the 
 * 5-point calibration table. It dynamically injects real-time values for stator 
 * flux linkage (psi), inductances (Ld, Lq, L), and inverse inductances directly into 
 * the running FOC structures based on the measured rotor field current.
 * 
 * @param  motor: Pointer to the active motor state structure containing telemetry.
 * @return None.
 */
static inline void wrsm_update_foc_parameters(motor_all_state_t *motor) {
    int target_row = MGU_LOOKUP_SECTORS - 1;
    float filtered_if = motor->m_field_current;

    for (int i = 0; i < MGU_LOOKUP_SECTORS - 1; i++) {
        if (mgu_if_table[i].lower_bound_if <= filtered_if) {
            target_row = i;
            break;
        }
    }

    const if_lookup_row_t *row = &mgu_if_table[target_row];
    float shared_delta = filtered_if - row->lower_bound_if;
    if (shared_delta < 0.0f) {
        shared_delta = 0.0f;
    }

    motor->m_injected_flux         = row->base_flux         + (shared_delta * row->slope_flux);
    motor->m_injected_ld           = row->base_ld           + (shared_delta * row->slope_ld);
    motor->m_injected_lq           = row->base_lq           + (shared_delta * row->slope_lq);
    motor->m_injected_l            = row->base_l            + (shared_delta * row->slope_l);
    motor->m_injected_ld_lq_diff   = row->base_ld_lq_diff   + (shared_delta * row->slope_ld_lq_diff);
    motor->m_injected_inv_ld       = row->base_inv_ld       + (shared_delta * row->slope_inv_ld);
    motor->m_injected_inv_lq       = row->base_inv_lq       + (shared_delta * row->slope_inv_lq);
    motor->m_injected_p_inv_ld_lq  = motor->m_injected_inv_lq - motor->m_injected_inv_ld;
    motor->m_injected_p_v2_v3_inv_avg_half = 0.45f * (motor->m_injected_inv_lq + motor->m_injected_inv_ld);
}

// --- Level 2 Control Interface ---

/**
 * @brief  Sets the raw PWM duty cycle of the physical rotor field H-bridge.
 * 
 * Safely truncates the incoming floating-point duty cycle to [0.0, 1.0], maps 
 * the value across the timer's hardware period width (200 ticks for the 5 kHz 
 * carrier frequency), and writes it directly to the STM32 TIM4 compare register.
 * 
 * @param  motor: Pointer to the active motor state structure.
 * @param  duty: Floating-point target duty cycle [0.0 to 1.0].
 * @return None.
 */
void wrsm_set_field_duty(motor_all_state_t *motor, float duty);

/**
 * @brief  Controls the physical gate-driver enable line of the field H-bridge.
 * 
 * Manages the active-high PB10 driver enable pin (HW_FIELD_EN_PIN). 
 * If enable is requested, it instantly asserts the pin unless a hardware fault 
 * lockout is active. If disable is requested, it zeroes the duty cycle to trigger 
 * the low-side soft-decay freewheeling sequence, leaving the physical shutdown 
 * handling to the 1 kHz timer thread.
 * 
 * @param  motor: Pointer to the active motor state structure.
 * @param  enable: True to assert gate driver; False to begin freewheeling shutdown.
 * @return None.
 */
void wrsm_set_field_enable(motor_all_state_t *motor, bool enable);

// --- Level 3 Control Interface ---
/**
 * @brief  Closed-loop PI regulator for WRSM rotor field current (I_f).
 * @note   Executed at 1 kHz inside the timer_update() background thread.
 * 
 * Resolves high-level supervisor overrides versus the optimal copper loss solver. 
 * Enforces live electrical boundaries to protect the vehicle:
 *   1. Battery-compensated ceiling limit (prevents PI integrator windup).
 *   2. Speed-compensated back-EMF safety clamp (prevents passive diode conduction).
 * Computes the error, runs the PI regulation, and updates the physical PWM duty cycle.
 * 
 * @param  motor: Pointer to the active motor state structure.
 * @param  dt: Integration timestep in seconds (0.001f for 1 kHz).
 * @return None.
 */
void wrsm_update_field_control(motor_all_state_t *motor, float dt);


#endif /* WRSM_FIELD_CONTROLLER_H_ */
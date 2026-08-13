/*
 * wrsm_field_controller.c
 *
 * Mathematically Optimized & Battery-Compensated Closed-Loop Rotor Field Current (I_f) Controller.
 *
 * This module implements the complete 1 kHz closed-loop PI field controller for Wound Rotor
 * Synchronous Motors (WRSM). It seamlessly handles:
 *   1. Supervisor Overrides (e.g., cold-start pre-excitation)
 *   2. Optimal Stator-Rotor Loss-Minimization Curve-Fitting
 *   3. Live Battery-Voltage Limits (Anti-Windup Protection)
 *   4. High-Speed Back-EMF Overvoltage Safeguards (diode drop safety margin)
 *   5. Direct ChibiOS Hardware Gating (BTS7960 Enable & PWM generation)
 *
 * Released under the GNU General Public License.
 */

#include "wrsm_field_controller.h"
#include "mcpwm_foc.h"
#include "foc_math.h"
#include "utils_math.h"
#include "hw.h"
#include <math.h>

// ChibiOS Headers for physical pin and timer manipulation
#include "ch.h"
#include "hal.h"

// Background WRSM Field PI controller state structure
static struct {
    float kp;
    float ki;
    float integrator;
    float prev_error;
} field_pid = {
    .kp = 0.15f,
    .ki = 1.50f,
    .integrator = 0.0f,
    .prev_error = 0.0f
};

// ============================================================================
// --- LEVEL 2 PHYSICAL DRIVER FUNCTIONS ---
// ============================================================================

void wrsm_set_field_duty(motor_all_state_t *motor, float duty) {
    if (motor->m_field_duty == duty) return;
    // 1. Clamp duty cycle between 0% and 100% for physical safety
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // 2. Map float [0.0 - 1.0] to Timer ARR ticks (200 ticks = 5kHz at 1MHz) [4]
    uint32_t width = (uint32_t)(duty * 200.0f);

    pwmEnableChannel(&PWMD4, 0, width);
    motor->m_field_duty = duty;
}

void wrsm_set_field_enable(motor_all_state_t *motor, bool enable) {
    motor->m_field_enable_request = enable;
    if (enable) {
        if (!motor->m_field_ESTOP_LOCKOUT) {
            palSetPad(HW_FIELD_EN_GPIO, HW_FIELD_EN_PIN); // Turn on immediately [5]
            motor->m_field_enable_pin_active = true;
        }
    } else {
        // Soft decay freewheeling sequence starts by setting duty to 0% [5]
        // The 1kHz timer thread (via wrsm_update_field_control) will physically clear the pin.
        wrsm_set_field_duty(motor, 0.0f);
    }
}

// ============================================================================
// --- LEVEL 3 REGULATOR LOOP (1kHz timer context) ---
// ============================================================================
void wrsm_update_field_control(motor_all_state_t *motor, float dt) {
    float measured_if = motor->m_field_current;
    // --- DYNAMIC POSITION OBSERVER GAIN NORMALIZATION ---
    // Keeps Ortega/MXLemming tracking bandwidth perfectly constant as rotor flux warps
    float live_flux = motor->m_injected_flux;
    if (live_flux > 0.001f) {
        motor->m_conf->foc_observer_gain = 1000.0f / (live_flux * live_flux);
    }

    // STATE 1: HARD ESTOP LOCKOUT (Permanent until power-cycle reboot)
    if (motor->m_field_ESTOP_LOCKOUT) {
        // We do no decay checks or pin toggles. ESTOP has already forced a shutdown.
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    // STATE 2: SOFT DISABLE / NON-ESTOP FAULT DECAY (Intent is OFF)
    if (!motor->m_field_enable_request) {
        // Zero the control target and integrator immediately
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;

        // Watch the physical enable pin to see if the gate driver is still active
        if (motor->m_field_enable_pin_active) {
            // Force 0% duty (keeps low-sides active to freewheel decay the inductor)
            wrsm_set_field_duty(motor, 0.0f);

            // Once the current is safely below our freewheeling limit, turn off the gate driver
            if (fabsf(measured_if) < 0.30f) {
                WRSM_FIELD_DISABLE(); // Physically pull PB10 LOW (High-Z)
                motor->m_field_enable_pin_active = false;
            }
        }
        return; // Exit immediately
    }

    // STATE 3: ENABLE REQUESTED (Intent is ON)
    // If the driver isn't physically turned on yet, we do no math at all.
    // The pin must be asserted by wrsm_set_field_enable(true) elsewhere.
    if (!motor->m_field_enable_pin_active) {
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    // ============================================================================
    // STATE 4: STANDBY / IDLE (MC_STATE_OFF but Intent is ON & Hardware is ON)
    // ============================================================================
    if (motor->m_state == MC_STATE_OFF) {
        wrsm_set_field_duty(motor, 0.0f); // Force 0% duty (active low-side freewheeling ready)
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    // ============================================================================
    // STATE 5: ACTIVE CLOSED-LOOP REGULATION (FOC Running, Intent is ON, HW is ON)
    // ============================================================================
    float target_if = 0.0f;

    // --- STEP A: RESOLVE OVERRIDE VS. LOSS-MINIMIZATION SOLVER ---
    if (motor->m_field_override_active) {
        target_if = motor->m_field_override_current;
    } else {
        float iq_target_abs = fabsf(motor->m_motor_state.iq_target);
        if (iq_target_abs > 0.1f) {
            // Optimal stator-rotor copper loss balancing
            target_if = 0.191f * sqrtf(iq_target_abs) - 0.087f;
        }
    }

    // --- STEP B: DYNAMIC VEHICLE ELECTRICAL LIMITS & SAFETY CLAMPS ---
    float erpm_abs = fabsf(mcpwm_foc_get_rpm());
    float v_batt = motor->m_motor_state.v_bus;

    // Limit A: Physical current ceiling based on battery voltage (prevents PI windup)
    float max_possible_if = (v_batt - 0.5f) / 5.60f;
    if (max_possible_if < 0.0f) max_possible_if = 0.0f;

    // Limit B: Back-EMF safety clamp (prevents passive diode conduction)
    float max_safe_if = (741.02f * (v_batt - 1.0f)) / fmaxf(erpm_abs, 100.0f) - 0.414f;
    if (max_safe_if < 0.0f) max_safe_if = 0.0f;

    // Apply the most restrictive limit (clamped at physical ceiling of 2.92A)
    float upper_limit = (max_possible_if < max_safe_if) ? max_possible_if : max_safe_if;
    float safe_ceiling = (upper_limit < 2.92f) ? upper_limit : 2.92f;
    utils_truncate_number(&target_if, 0.0f, safe_ceiling);

    motor->m_field_current_target = target_if;

    // --- STEP C: CLOSED-LOOP PI REGULATION ---
    float error = target_if - measured_if;
    float p_term = error * field_pid.kp;

    // Anti-Windup: Freeze integration if duty is saturated and error is positive
    if (motor->m_field_duty >= 1.0f && error > 0.0f) {
        // Freeze integration
    } else {
        field_pid.integrator += error * field_pid.ki * dt;
    }
    utils_truncate_number(&field_pid.integrator, -0.1f, 1.0f);

    float duty_out = p_term + field_pid.integrator;
    utils_truncate_number(&duty_out, 0.0f, 1.0f);

    // --- STEP D: PHYSICAL PWM HARDWARE UPDATE ---
    wrsm_set_field_duty(motor, duty_out);
    field_pid.prev_error = error;
}
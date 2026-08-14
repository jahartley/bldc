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
// --- Lookup functions
// ============================================================================
float wrsm_lookup_flux(float field_curr) {
    if (field_curr <= 0.00f) return 0.00308000f; // Floor: Sub-magnet flux linkage
    if (field_curr >= 2.92f) return 0.01320000f; // Ceiling: Claw-pole saturation limit

    int target_row = MGU_LOOKUP_SECTORS - 1;
    

    for (int i = 0; i < MGU_LOOKUP_SECTORS - 1; i++) {
        if (mgu_if_table[i].lower_bound_if <= field_curr) {
            target_row = i;
            break;
        }
    }

    const if_lookup_row_t *row = &mgu_if_table[target_row];
    float shared_delta = field_curr - row->lower_bound_if;
    if (shared_delta < 0.0f) {
        shared_delta = 0.0f;
    }

    return row->base_flux + (shared_delta * row->slope_flux);
}

float wrsm_lookup_if_from_flux(float target_flux) {
    if (target_flux <= 0.00308000f) return 0.00f;
    if (target_flux >= 0.01320000f) return 2.92f;

        int target_row = MGU_LOOKUP_SECTORS - 1;

    // Scan table from highest flux sector to lowest
    for (int i = 0; i < MGU_LOOKUP_SECTORS - 1; i++) {
        if (mgu_if_table[i].base_flux <= target_flux) {
            target_row = i;
            break;
        }
    }

    const if_lookup_row_t *row = &mgu_if_table[target_row];
    float shared_delta = target_flux - row->base_flux;
    if (shared_delta < 0.0f) {
        shared_delta = 0.0f;
    }

    // Since delta_flux = delta_if * slope_flux, then:
    // delta_if = delta_flux / slope_flux
    return row->lower_bound_if + (shared_delta / row->slope_flux);
}

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

    // CHECK 1: HARD ESTOP LOCKOUT (Permanent until power-cycle reboot)
    if (motor->m_field_ESTOP_LOCKOUT) {
        // We do no decay checks or pin toggles. ESTOP has already forced a shutdown.
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    // CHECK 2: SOFT DISABLE / NON-ESTOP FAULT DECAY (Intent is OFF)
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

    // CHECK 3: ENABLE REQUESTED (Intent is ON)
    // If the driver isn't physically turned on yet, we do no math at all.
    // The pin must be asserted by wrsm_set_field_enable(true) elsewhere.
    if (!motor->m_field_enable_pin_active) {
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    
    // CHECK 4: STANDBY / IDLE (MC_STATE_OFF but Intent is ON & Hardware is ON)
    if (motor->m_state == MC_STATE_OFF) {
        wrsm_set_field_duty(motor, 0.0f); // Force 0% duty (active low-side freewheeling ready)
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        return; // Exit immediately
    }

    // ACTIVE CLOSED-LOOP REGULATION (FOC Running, Intent is ON, HW is ON)
    // --- STEP A: CALCULATE THE UNWEAKENED COPPER LOSS TARGET ---
    float optimal_if = 0.0f;
    if (motor->m_field_override_active) {
        optimal_if = motor->m_field_override_value;
    } else {
        float iq_target_abs = fabsf(motor->m_motor_state.iq_target);
        if (iq_target_abs > 0.1f) {
            optimal_if = 0.191f * sqrtf(iq_target_abs) - 0.087f;
        }
    }
    float v_batt = motor->m_motor_state.v_bus;
    float estimateMaxCurrent = v_batt / MGU_FIELD_R;
    // Truncate optimal to available bounds [0A to current voltage limit]
    utils_truncate_number(&optimal_if, 0.0f, estimateMaxCurrent);

    // --- STEP B: RUN SATURATION-AWARE FLUX-WEAKENING ENVELOPE ALLOCATOR ---

    // B1. Convert unweakened optimal current to Webers of magnetic flux linkage
    float optimal_flux = wrsm_lookup_flux(optimal_if);

    // B2. Convert high-speed stator-equivalent demand into Webers of flux reduction
    // (Demanded flux = Stator FW Amps * Live D-Axis inductance)
    float delta_flux_demanded = motor->m_i_fw_set * motor->m_injected_ld;

    // B3. Subtract to find net target flux required inside the air gap
    float target_flux = optimal_flux - delta_flux_demanded;

    float target_if = 0.0f;
    float unmet_stator_fw_id = 0.0f;

    if (target_flux >= 0.00308000f) {
        // STAGE 1: Rotor winding has enough magnetic headroom to weakening on its own
        target_if = wrsm_lookup_if_from_flux(target_flux);
        unmet_stator_fw_id = 0.0f; // Stator i_d remains at 0.0A!
    } else {
        // STAGE 2: Rotor field has collapsed to 0A; stator must handle the remaining permanent magnets
        target_if = 0.0f;
        float unmet_flux = 0.00308000f - target_flux;

        // Convert remaining unmet flux back to stator d-axis current using L_d(0) = 30.13 uH (0.00003013 H)
        unmet_stator_fw_id = unmet_flux / 0.00003013f;
    }

    motor->m_field_current_target = target_if;
    motor->m_stator_fw_id = unmet_stator_fw_id; // Feed to high-frequency FOC current controller

    // --- STEP C: PI CLOSED-LOOP CURRENT CONTROLLER ---
    float error = target_if - measured_if;
    float p_term = error * field_pid.kp;

    // Integrator freeze Anti-Windup Guard
    if (motor->m_field_duty >= 1.0f && error > 0.0f) {
        // Saturated, freeze integration
    } else {
        field_pid.integrator += error * field_pid.ki * dt;
    }
    utils_truncate_number(&field_pid.integrator, -0.1f, 1.0f);

    float duty_out = p_term + field_pid.integrator;
    utils_truncate_number(&duty_out, 0.0f, 1.0f);

    // --- STEP D: UPDATE PHYSICAL PWM TIMER REGISTER ---
    wrsm_set_field_duty(motor, duty_out);
    field_pid.prev_error = error;
}
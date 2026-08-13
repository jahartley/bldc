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

// Global override flags accessed asynchronously by the background 1 kHz supervisor thread
volatile bool field_override_active = false;
volatile float field_override_value = 0.0f;

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

/**
 * Executes the 1 kHz closed-loop rotor field current PI regulator.
 * Calculates optimal losses or processes overrides, enforces limits, and gates physical pins.
 *
 * Call this inside timer_update() in mcpwm_foc.c.
 *
 * @param motor Pointer to the global motor state structure.
 * @param dt Timestep in seconds (0.001f for 1 kHz timer loop).
 */
void wrsm_update_field_control(motor_all_state_t *motor, float dt) {
    mc_configuration *conf_now = motor->m_conf;
    
    // Safety Interlock: If FOC is disabled or in a fault state, shut down the H-bridge immediately
    if (motor->m_state == MC_STATE_OFF) {
        // Force H-bridge enable pin LOW
        palClearPad(HW_FIELD_EN_GPIO, HW_FIELD_EN_PIN);
        // Force PWM duty cycle to 0.0%
        pwmEnableChannel(&PWMD4, 0, 0);
        
        // Reset controller states to prevent start-up voltage spikes
        field_pid.integrator = 0.0f;
        field_pid.prev_error = 0.0f;
        motor->m_field_current_target = 0.0f;
        motor->m_field_duty = 0.0f;
        return;
    }
    
    // Assert physical enable pin on the BTS7960 H-bridge
    palSetPad(HW_FIELD_EN_GPIO, HW_FIELD_EN_PIN);
    
    float target_if = 0.0f;
    
    // --- STEP 1: RESOLVE OVERRIDE VS. LOSS-MINIMIZATION SOLVER ---
    if (field_override_active) {
        // High-level supervisor has manually seized control (e.g., during start-up or stall-recovery)
        target_if = field_override_value;
    } else {
        // Native optimal stator-rotor copper loss balancing
        float iq_target_abs = fabsf(motor->m_motor_state.iq_target);
        if (iq_target_abs > 0.1f) {
            // Highly optimized non-linear square-root curve fit
            target_if = 0.191f * sqrtf(iq_target_abs) - 0.087f;
        } else {
            target_if = 0.0f; // Collapse field completely at standby to save battery
        }
    }
    
    // --- STEP 2: DYNAMIC VEHICLE ELECTRICAL LIMITS & SAFETY CLAMPS ---
    float erpm_abs = fabsf(mcpwm_foc_get_rpm());
    float v_batt = motor->m_motor_state.v_bus;
    
    // Limit A: Physical current ceiling based on live battery voltage (prevents PI windup)
    // Rf = 5.60 Ohms. V_drop across BTS7960 silicon under load ~ 0.5V.
    float max_possible_if = (v_batt - 0.5f) / 5.60f;
    if (max_possible_if < 0.0f) max_possible_if = 0.0f;
    
    // Limit B: Back-EMF overvoltage safety clamp (prevents passive diode conduction)
    // Ensures peak line-to-line EMF remains at least 1.0V below live battery voltage.
    float max_safe_if = (741.02f * (v_batt - 1.0f)) / fmaxf(erpm_abs, 100.0f) - 0.414f;
    if (max_safe_if < 0.0f) max_safe_if = 0.0f;
    
    // Select the most restrictive upper limit to safeguard stator electronics
    float upper_limit = (max_possible_if < max_safe_if) ? max_possible_if : max_safe_if;
    
    // Truncate target field current to safe limits (physical winding ceiling = 2.92A)
    float safe_ceiling = (upper_limit < 2.92f) ? upper_limit : 2.92f;
    utils_truncate_number(&target_if, 0.0f, safe_ceiling);
    
    motor->m_field_current_target = target_if;
    
    // --- STEP 3: CLOSED-LOOP PI REGULATION ---
    // Read the 15 kHz filtered rotor current updated by the high-speed ADC interrupt
    float measured_if = motor->m_field_current;
    
    float error = target_if - measured_if;
    
    // Proportional Term
    float p_term = error * field_pid.kp;
    
    // Integral Term with Anti-Windup Clamping
    // If duty cycle is already saturated at 100% and error remains positive, freeze integration
    if (motor->m_field_duty >= 1.0f && error > 0.0f) {
        // Freeze integration (anti-windup)
    } else {
        field_pid.integrator += error * field_pid.ki * dt;
    }
    utils_truncate_number(&field_pid.integrator, -0.1f, 1.0f);
    
    // Calculate total control effort (Duty Cycle)
    float duty_out = p_term + field_pid.integrator;
    utils_truncate_number(&duty_out, 0.0f, 1.0f);
    
    // --- STEP 4: PHYSICAL PWM HARDWARE UPDATE ---
    // TIM4 setup parameters in conf_general.c configure period to exactly 200 ticks (5 kHz)
    uint32_t compare_width = (uint32_t)(duty_out * 200.0f);
    pwmEnableChannel(&PWMD4, 0, compare_width); // PB6 (Channel 0 on PWMD4)
    
    // Write back active state for telemetry & VESC Tool graphing
    motor->m_field_duty = duty_out;
    field_pid.prev_error = error;
}
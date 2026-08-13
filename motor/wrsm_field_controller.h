#ifndef WRSM_FIELD_CONTROLLER_H_
#define WRSM_FIELD_CONTROLLER_H_

#include "foc_math.h"

// ============================================================================
// --- JAH: WRSM ROTOR FIELD REGULATION INTERFACE ---
// ============================================================================

/**
 * Global override flags. 
 * Your background 1 kHz supervisor thread will manipulate these to bypass 
 * the optimal loss-minimizer (e.g., during cold-start pre-excitation).
 */
extern volatile bool field_override_active;
extern volatile float field_override_value;

/**
 * 1 kHz Closed-Loop Rotor Field Controller.
 * Executes the battery-voltage-compensated PI loop to regulate the H-bridge duty cycle.
 * 
 * Call this inside timer_update() in mcpwm_foc.c.
 * 
 * @param motor Pointer to the global motor state structure containing WRSM telemetry.
 * @param dt Timestep in seconds (0.001f for the 1 kHz timer loop).
 */
void wrsm_update_field_control(motor_all_state_t *motor, float dt);

#endif /* WRSM_FIELD_CONTROLLER_H_ */
#ifndef WRSM_SUPERVISOR_H_
#define WRSM_SUPERVISOR_H_

#include "foc_math.h"
#include <stdbool.h>

/**
 * @brief Level 4 WRSM Vehicle Supervisor States
 * 
 * Defines the state space for coordinating the transition of the Wound Rotor 
 * Synchronous Machine between motoring (engine cranking) and generating (alternator) modes.
 */
typedef enum {
    WRSM_SUPER_STATE_OFF = 0,       // Zero RPM, rotor unexcited, stator phases off.
    WRSM_SUPER_STATE_BOOT,          // Startup state. MGU may be spinning.
    WRSM_SUPER_STATE_STOPPING,      // STATE_OFF desired, MGU still spinning.
    WRSM_SUPER_STATE_PRE_EXCITE,    // Pre-energizing rotor coil to establish starting magnetic flux
    WRSM_SUPER_STATE_CRANKING,      // Active engine cranking (high-torque motoring mode)
    WRSM_SUPER_STATE_ALTERNATOR,    // Engine running; MGU regulating bus voltage/charging battery
    WRSM_SUPER_STATE_STALL_CATCH,   // Dynamic MGU assist to prevent engine stall.
    WRSM_SUPER_STATE_FAULT,         // Recoverable system fault (e.g., transient thermal warning, CAN drop)
    WRSM_SUPER_STATE_ESTOP          // Terminal latching hardware emergency stop (requires power cycle)
} wrsm_super_state_t;

// ============================================================================
// --- CORE SUPERVISOR API ---
// ============================================================================

/**
 * @brief Initializes the supervisor state machine.
 * 
 * Performs startup speed checks to decide whether to boot into OFF (stationary)
 * or hot-boot directly into ALTERNATOR mode (if the engine is already running).
 * 
 * @param[in,out] motor Pointer to the core motor state structure.
 */
void wrsm_supervisor_init(motor_all_state_t *motor);

/**
 * @brief High-level supervisor tick running at 1 kHz in the background.
 * 
 * Evaluates state transitions, safety interlocks, and updates lower-level 
 * overrides for the rotor field current and stator control modes.
 * 
 * @param[in,out] motor Pointer to the core motor state structure.
 * @param[in] dt Time elapsed since the last tick in seconds (typically 0.001f).
 */
void wrsm_supervisor_update(motor_all_state_t *motor, float dt);

/**
 * @brief Safely requests a transition to a new supervisor state.
 * 
 * This function enforces state transition logic and guards, ensuring illegal
 * or dangerous state jumps (like escaping an active ESTOP) are blocked.
 * 
 * @param[in,out] motor Pointer to the core motor state structure.
 * @param[in] requested_state The target state to transition into.
 */
void wrsm_supervisor_request_state(motor_all_state_t *motor, wrsm_super_state_t requested_state);

/**
 * @brief Retrieves the active supervisor state.
 * 
 * @return The current active wrsm_super_state_t enum value.
 */
wrsm_super_state_t wrsm_supervisor_get_state(void);

/**
 * @brief Converts a supervisor state enum into a human-readable string.
 * 
 * Useful for serial diagnostics, CAN status broadcasting, and debugging.
 * 
 * @param[in] state The state enum to convert.
 * @return A constant null-terminated string representing the state name.
 */
const char* wrsm_supervisor_state_to_str(wrsm_super_state_t state);

#endif /* WRSM_SUPERVISOR_H_ */
#include "wrsm_supervisor.h"
#include "wrsm_field_controller.h"
#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "hw.h"
#include "ch.h"
#include "hal.h"
#include <math.h>

// --- STATE MACHINE CONSTANTS (MGU MECHANICAL DOMAIN) ---
#define MGU_MIN_ALTERNATOR_RPM      1500.0f // minimum MGU rpm to set alternator mode
#define MGU_START_TARGET_RPM        600.0f  // Start mode target RPM
#define MGU_RPM_RUNNING_THRESHOLD   2400.0f // MGU RPM above which power generation is active
#define MGU_RPM_STALL_THRESHOLD     1200.0f // MGU RPM below which stall-recovery is triggered
#define MGU_RPM_STOPPED_THRESHOLD   10.0f  // MGU RPM below which MGU is considered stationary
#define CRANK_TIMEOUT_SEC           10.0f   // Maximum allowed cranking duration at start speed
#define CRANK_RAMP_SEC              3.0f    // Duration to ramp to start speed
#define PRE_EXCITE_TIMEOUT_SEC      0.15f   // Maximum allowed time to build rotor flux
#define STALL_RECOVERY_TIMEOUT      1.5f    // Maximum duration to attempt a flying recovery

// --- STATE HANDLER STRUCTURE DEFINITION ---
typedef struct {
    wrsm_super_state_t (*guard)(motor_all_state_t *motor, wrsm_super_state_t from_state);
    void (*on_entry)(motor_all_state_t *motor);
    wrsm_super_state_t (*on_tick)(motor_all_state_t *motor, float dt);
} wrsm_super_state_handler_t;

// State-local timer used to track timeouts and transient intervals
static float state_timer = 0.0f;
static motor_all_state_t *m_active_motor = NULL;

static wrsm_super_state_t current_state = WRSM_SUPER_STATE_BOOT;   // The actual active state
static wrsm_super_state_t requested_state = WRSM_SUPER_STATE_BOOT; // The mailbox target

static float pole_pairs = 201.0f;

// ============================================================================
// --- STATE-SPECIFIC HANDLERS ---
// ============================================================================

// --- STATE: OFF / STANDBY ---
static void state_off_entry(motor_all_state_t *motor) {
    //wrsm_set_field_enable(motor, false);
    motor->m_field_override_active = true; //should be off, take field ownership.
    motor->m_field_override_current = 0.0f;
    mcpwm_foc_stop_pwm(false);
}

static wrsm_super_state_t state_off_tick(motor_all_state_t *motor, float dt) {
    // Standby: Wait for external transition request or check motor spin.
    float current_erpm = fabsf(mcpwm_foc_get_rpm());
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * pole_pairs;
    
    // automatic move to alternator mode prevents letting stator current flow 
    // through body diodes due to spin + base flux.
    if (current_erpm > erpm_threshold) {
        motor->m_field_override_active = false; // let automatic field control run.
        return WRSM_SUPER_STATE_ALTERNATOR;
    }
    return WRSM_SUPER_STATE_OFF;
}

// --- STATE: BOOT ---
static void state_boot_entry(motor_all_state_t *motor) { // This should never actually run.
    wrsm_set_field_enable(motor, true);
    motor->m_field_override_active = false; // we may be already spinning or hot restart. let auto field have control.
    motor->m_field_override_current = 0.0f;
}

static wrsm_super_state_t state_boot_tick(motor_all_state_t *motor, float dt) {
    wrsm_set_field_enable(motor, true); // no specific reason this should be off.
    float current_erpm = fabsf(mcpwm_foc_get_rpm());
    if (pole_pairs > 200.0f) { // no need to do this every pass...
        // Set pole pairs value by lookup on boot
        pole_pairs = (float)motor->m_conf->si_motor_poles / 2.0f;
        if (pole_pairs < 1.0f) { // divide by zero protection.
            pole_pairs = 8.0f;
        }
    }
    
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * pole_pairs;
    // transition off boot after first pass through boot tick
    if (current_erpm > erpm_threshold) {
        motor->m_field_override_active = false; // let automatic field control run.
        return WRSM_SUPER_STATE_ALTERNATOR;
    }
    return WRSM_SUPER_STATE_OFF;
}

// --- STATE: ENGINE_STOPPING (COAST-DOWN TO STANDBY) ---
static void state_stopping_entry(motor_all_state_t *motor) {
    motor->m_field_override_active = true; // goal is to stop, take field control back.
    motor->m_field_override_current = 0.0f;
    mcpwm_foc_set_current(0.0f); // make motor freewheel without killing fet diodes.
}

static wrsm_super_state_t state_stopping_tick(motor_all_state_t *motor, float dt) {
    // watch for FOC Control to transistion to MC_STATE_OFF/CONTROL_MODE_NONE at minimum ERPM

    if (motor->m_state == MC_STATE_OFF || motor->m_control_mode == CONTROL_MODE_NONE) {
        return WRSM_SUPER_STATE_OFF; // Stator shut down complete, go to off.
    }

    return WRSM_SUPER_STATE_STOPPING; // Continue waiting.
}

// --- STATE: PRE_EXCITATION ---
static wrsm_super_state_t state_pre_excite_guard(motor_all_state_t *motor, wrsm_super_state_t from_state) {
    if (from_state != WRSM_SUPER_STATE_OFF) { // verify correct transition from off.
        if (from_state == WRSM_SUPER_STATE_ALTERNATOR || 
            from_state == WRSM_SUPER_STATE_STALL_CATCH) {
            return from_state; // Transition rejected, stay where we are!
        } else {
            // 2. Otherwise, it is a sequencing violation (e.g. from OFF) -> Route to FAULT
            return WRSM_SUPER_STATE_FAULT;
        }
    }
    // Prevent entering PRE_EXCITE if stator control is not off/stopped.
    if (!(motor->m_state == MC_STATE_OFF && motor->m_control_mode == CONTROL_MODE_NONE)) {
        return WRSM_SUPER_STATE_FAULT;
    }
    // RPM Guard: Prevent entering PRE_EXCITE if the MGU is already spinning
    float current_erpm = fabsf(mcpwm_foc_get_rpm());
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * pole_pairs;
    
    if (current_erpm > erpm_threshold) {
        return WRSM_SUPER_STATE_FAULT;        
    }
    
    // 3. Safe to proceed to pre-excitation
    return WRSM_SUPER_STATE_PRE_EXCITE;
}

static void state_pre_excite_entry(motor_all_state_t *motor) {
    // GO FULL FIELD (actual max amperage will depend on batt voltage)
    wrsm_set_field_enable(motor, true);
    motor->m_field_override_active = true;
    motor->m_field_override_current = 2.50f; // 2.5A starting excitation target
}

static wrsm_super_state_t state_pre_excite_tick(motor_all_state_t *motor, float dt) {
    // Wait out the field building delay.
    if (state_timer >= PRE_EXCITE_TIMEOUT_SEC) {
        return WRSM_SUPER_STATE_CRANKING;
    }
    return WRSM_SUPER_STATE_PRE_EXCITE;
}

// --- STATE: ENGINE_CRANKING ---
static wrsm_super_state_t state_cranking_guard(motor_all_state_t *motor, wrsm_super_state_t from_state) {
    // RPM Guard: Prevent entering CRANKING from anywhere other than PRE_EXCITE
    if (from_state != WRSM_SUPER_STATE_PRE_EXCITE) return WRSM_SUPER_STATE_FAULT;
    return WRSM_SUPER_STATE_CRANKING;
}

static void state_cranking_entry(motor_all_state_t *motor) {
    // rotor current should be maxed already, give currnent control back to the automatic setting.
    motor->m_field_override_active = false;
    motor->m_field_override_current = 0.0f;

    float target_erpm = MGU_START_TARGET_RPM * pole_pairs;

    // Calculate required acceleration rate (Delta ERPM / Time)
    float required_ramp_rate = target_erpm / CRANK_RAMP_SEC;

    // Apply calculated ramp rate dynamically to configuration
    motor->m_conf->s_pid_ramp_erpms_s = required_ramp_rate;

    // Command the speed PID loop to execute
    mcpwm_foc_set_pid_speed(target_erpm);
}

static wrsm_super_state_t state_cranking_tick(motor_all_state_t *motor, float dt) {
    float current_mgu_erpm = fabsf(mcpwm_foc_get_rpm());
    float started_threshold_erpm = MGU_START_TARGET_RPM * pole_pairs * 1.2f; //120% of start erpm

    if (current_mgu_erpm > started_threshold_erpm) { // Already beyond 120% target_erpm, must be driven by engine.
        return WRSM_SUPER_STATE_ALTERNATOR;
    } else if (state_timer >= CRANK_TIMEOUT_SEC) { // Max start time elapsed.
        return WRSM_SUPER_STATE_STOPPING;
    }
    return WRSM_SUPER_STATE_CRANKING;
}

// --- STATE: ALTERNATOR_ACTIVE ---
static void state_alternator_entry(motor_all_state_t *motor) {
    motor->m_field_override_active = false; // Relinquish manual control to optimal loss curves
}

static wrsm_super_state_t state_alternator_tick(motor_all_state_t *motor, float dt) {
    // check for rpm drop so low that stator has turned off unexpectedly.
    if (!(motor->m_state == MC_STATE_OFF && motor->m_control_mode == CONTROL_MODE_NONE)) {
        //return WRSM_SUPER_STATE_FAULT;
        return WRSM_SUPER_STATE_OFF;
    }
    
    // JAHTODO make stall catch decision here.
    float current_mgu_erpm = fabsf(mcpwm_foc_get_rpm());
    if (false) {
        return WRSM_SUPER_STATE_STALL_CATCH;
    }
    
    // JAHTODO REGULATE CURRENT PRODUCTION HERE!!!
    // motor->m_iq_set //current requested current.

    // JAHTODO taper alternator strength below minimum erpm
        
    return WRSM_SUPER_STATE_ALTERNATOR;
}

// --- STATE: STALL_CATCH ---
static void state_stall_catch_entry(motor_all_state_t *motor) {
    return;
}

static wrsm_super_state_t state_stall_catch_tick(motor_all_state_t *motor, float dt) {
    // JAHTODO add actual code for stall catch
    // set stall catch speed pid accel rate, should be higher than start, or even off.
    // switch to mcpwm_foc_set_pid_speed(calc target)
    
    // JAHTODO find all / pole_pairs antipattern and do the multiply from state off pattern. 
    float current_mgu_rpm = fabsf(mcpwm_foc_get_rpm()) / pole_pairs;

    if (current_mgu_rpm > MGU_RPM_RUNNING_THRESHOLD) {
        return WRSM_SUPER_STATE_ALTERNATOR;
    } else if (current_mgu_rpm < MGU_RPM_STOPPED_THRESHOLD || state_timer >= STALL_RECOVERY_TIMEOUT) {
        return WRSM_SUPER_STATE_OFF;
    }
    return WRSM_SUPER_STATE_STALL_CATCH;
}

// --- STATE: NORMAL_FAULT (RECOVERABLE) ---
static void state_fault_entry(motor_all_state_t *motor) {
    motor->m_field_override_active = true; //take field control.
    motor->m_field_override_current = 0.0f;
}

static wrsm_super_state_t state_fault_tick(motor_all_state_t *motor, float dt) {
    // Check if the stator fault condition has cleared
    if (mc_interface_get_fault() == FAULT_CODE_NONE) {
        
        float current_erpm = fabsf(mcpwm_foc_get_rpm());
        float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * pole_pairs;
        
        // automatic move to alternator mode prevents letting stator current flow 
        // through body diodes due to spin + base flux.
        if (current_erpm > erpm_threshold) {
            motor->m_field_override_active = false; // let automatic field control run.
            return WRSM_SUPER_STATE_ALTERNATOR;
        } else {
            return WRSM_SUPER_STATE_OFF;
        }
    }
    return WRSM_SUPER_STATE_FAULT;
}

// --- STATE: ESTOP_LOCKOUT (UNRECOVERABLE) ---
static void state_estop_entry(motor_all_state_t *motor) {
    WRSM_FIELD_DISABLE(); // Rapid hardware gate-driver disable via physical pin (PB10)
    motor->m_field_enable_pin_active = false;
    
    wrsm_set_field_enable(motor, false);
    motor->m_field_override_active = true;
    motor->m_field_override_current = 0.0f;
}

static wrsm_super_state_t state_estop_tick(motor_all_state_t *motor, float dt) {
    return WRSM_SUPER_STATE_ESTOP; // Lockout state has no soft exits
}

// ============================================================================
// --- STATE TABLE DEFINITION ---
// ============================================================================
static const wrsm_super_state_handler_t state_table[] = {
    [WRSM_SUPER_STATE_OFF] = {
        .guard = NULL,
        .on_entry = state_off_entry,
        .on_tick = state_off_tick
    },
    [WRSM_SUPER_STATE_BOOT] = {
        .guard = NULL,
        .on_entry = state_boot_entry,
        .on_tick = state_boot_tick
    },
    [WRSM_SUPER_STATE_STOPPING] = {
        .guard = NULL,
        .on_entry = state_stopping_entry,
        .on_tick = state_stopping_tick
    },
    [WRSM_SUPER_STATE_PRE_EXCITE] = {
        .guard = state_pre_excite_guard,
        .on_entry = state_pre_excite_entry,
        .on_tick = state_pre_excite_tick
    },
    [WRSM_SUPER_STATE_CRANKING] = {
        .guard = state_cranking_guard,
        .on_entry = state_cranking_entry,
        .on_tick = state_cranking_tick
    },
    [WRSM_SUPER_STATE_ALTERNATOR] = {
        .guard = NULL,
        .on_entry = state_alternator_entry,
        .on_tick = state_alternator_tick
    },
    [WRSM_SUPER_STATE_STALL_CATCH] = {
        .guard = NULL,
        .on_entry = state_stall_catch_entry,
        .on_tick = state_stall_catch_tick
    },
    [WRSM_SUPER_STATE_FAULT] = {
        .guard = NULL,
        .on_entry = state_fault_entry,
        .on_tick = state_fault_tick
    },
    [WRSM_SUPER_STATE_ESTOP] = {
        .guard = NULL,
        .on_entry = state_estop_entry,
        .on_tick = state_estop_tick
    }
};

static void process_internal_transition(motor_all_state_t *motor) {
    // --- 1. GLOBAL TRANSITION MATRIX OVERRIDES ---
    // If we are currently in a soft FAULT, we can only recover to Standby OFF 
    // or run a flying restart straight into ALTERNATOR.
    if (current_state == WRSM_SUPER_STATE_FAULT) {
        if (requested_state != WRSM_SUPER_STATE_OFF && requested_state != WRSM_SUPER_STATE_ALTERNATOR) {
            requested_state = current_state; // Reject the target and clear the mailbox
            return;
        }
    }

    wrsm_super_state_t resolved_state = requested_state;

    // --- 2. RUN TARGET STATE GUARD ---
    if (state_table[requested_state].guard) {
        resolved_state = state_table[requested_state].guard(motor, current_state);
    }

    // --- 3. PROCESS GUARD RESULTS ---
    
    // Case A: Guard completely rejected the transition, wanting us to stay where we are
    if (resolved_state == current_state) {
        requested_state = current_state; // Reset the mailbox target to match reality
        return;
    }

    // Case B: Guard redirected us to a different state (e.g., PRE_EXCITE redirected to FAULT)
    if (resolved_state != requested_state) {
        requested_state = resolved_state; // Post the redirected state back to the mailbox
        return; // EXIT EARLY! The next 1 kHz tick will evaluate this redirected state's guard
    }

    // Case C: Transition officially validated and accepted!
    // Execute the target state's one-shot entry actions
    if (state_table[resolved_state].on_entry) {
        state_table[resolved_state].on_entry(motor);
    }

    // Finalize state variables
    current_state = resolved_state;
    state_timer = 0.0f; // Clear state timer for the newly entered state
}


// ============================================================================
// --- CORE SUPERVISOR IMPLEMENTATION ---
// ============================================================================

void wrsm_supervisor_init(motor_all_state_t *motor) {
    m_active_motor = motor;
    state_timer = 0.0f;
    float pole_pairs_val = (float)motor->m_conf->si_motor_poles / 2.0f;
    if (pole_pairs_val > 0.0f) pole_pairs = pole_pairs_val;
}

void wrsm_supervisor_update(motor_all_state_t *motor, float dt) {
    m_active_motor = motor;
    state_timer += dt;

    // --- GLOBAL SAFETY TRANSITION INTERLOCKS ---
    // 1. Terminal Hardware ESTOP Check (Absolute dead-end)
    if (motor->m_field_ESTOP_LOCKOUT) {
        requested_state = WRSM_SUPER_STATE_ESTOP;
    }

    // 2. Recoverable Inverter/Stator Fault Check (Normal recoverable fault)
    if (mc_interface_get_fault() != FAULT_CODE_NONE && current_state != WRSM_SUPER_STATE_ESTOP) {
        requested_state = WRSM_SUPER_STATE_FAULT;
    }

    // --- THE TIMING SCHEDULER PARTITION ---
    if (current_state != requested_state) {
        // Run the transition evaluator and skip this tick's active update logic
        process_internal_transition(motor);
    } else {
        // Run the normal, steady-state update logic for the active state
        if (state_table[current_state].on_tick) {
            wrsm_super_state_t next_state = state_table[current_state].on_tick(motor, dt);
            
            // If the state tick decides it is time to leave, post to the mailbox
            if (next_state != current_state) {
                requested_state = next_state;
            }
        }
    }
}

void wrsm_supervisor_request_state(wrsm_super_state_t new_requested_state) {
    motor_all_state_t *motor = m_active_motor;

    if (current_state == new_requested_state) {
        return;
    }

    // --- GLOBAL TRANSITION MATRIX GUARDS ---
    
    // 1. ESTOP is terminal. Escape is impossible without physical power cycle.
    if (current_state == WRSM_SUPER_STATE_ESTOP) {
        return; 
    }

    // 2. State-Specific Guard Check
    wrsm_super_state_t resolved_state = requested_state;
    if (state_table[requested_state].guard) {
        resolved_state = state_table[requested_state].guard(motor, current_state);
    }

    // If the guard decided to stay where we are (resolved == current), abort transition
    if (resolved_state == current_state) {
        return;
    }

    // Execute state entry action EXACTLY ONCE
    if (state_table[requested_state].on_entry) {
        state_table[requested_state].on_entry(motor);
    }

    // Accept transition and clear local timer
    current_state = requested_state;
    state_timer = 0.0f;
}

wrsm_super_state_t wrsm_supervisor_get_state(void) {
    return current_state;
}

const char* wrsm_supervisor_state_to_str(wrsm_super_state_t state) {
    switch (state) {
        case WRSM_SUPER_STATE_OFF:          return "STANDBY_OFF";
        case WRSM_SUPER_STATE_BOOT:         return "STANDBY_BOOT";
        case WRSM_SUPER_STATE_STOPPING:     return "STANDBY_SPINNING";
        case WRSM_SUPER_STATE_PRE_EXCITE:   return "PRE_EXCITATION";
        case WRSM_SUPER_STATE_CRANKING:     return "ENGINE_CRANKING";
        case WRSM_SUPER_STATE_ALTERNATOR:   return "ALTERNATOR_ACTIVE";
        case WRSM_SUPER_STATE_STALL_CATCH:  return "STALL_CATCH";
        case WRSM_SUPER_STATE_FAULT:        return "NORMAL_FAULT";
        case WRSM_SUPER_STATE_ESTOP:        return "ESTOP_LOCKOUT";
        default:                            return "UNKNOWN";
    }
}

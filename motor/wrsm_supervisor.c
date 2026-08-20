#include "wrsm_supervisor.h"
#include "wrsm_field_controller.h"
#include "mcpwm_foc.h"
#include "foc_math.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "hw.h"
#include "ch.h"
#include "hal.h"
#include <math.h>

// --- STATE MACHINE CONSTANTS (MGU MECHANICAL DOMAIN) ---
#define MGU_MIN_ALTERNATOR_RPM      1500.0f // minimum MGU rpm to set alternator mode
//#define MGU_START_TARGET_RPM        600.0f  // Start mode target RPM
#define MGU_RPM_RUNNING_THRESHOLD   2400.0f // MGU RPM above which power generation is active
#define MGU_RPM_STALL_THRESHOLD     1200.0f // MGU RPM below which stall-recovery is triggered
#define MGU_RPM_STOPPED_THRESHOLD   10.0f  // MGU RPM below which MGU is considered stationary
#define CRANK_TIMEOUT_SEC           30.0f   // Maximum allowed cranking duration at start speed
//#define CRANK_RAMP_SEC              3.0f    // Duration to ramp to start speed
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
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * POLE_PAIRS;
    
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
        
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * POLE_PAIRS;
    // transition off boot after first pass through boot tick
    if (current_erpm > erpm_threshold) {
        motor->m_field_override_active = false; // let automatic field control run.
        return WRSM_SUPER_STATE_ALTERNATOR;
    }
    return WRSM_SUPER_STATE_OFF;
}

static wrsm_super_state_t state_stopping_guard(motor_all_state_t *motor, wrsm_super_state_t from_state) {
    if (from_state == WRSM_SUPER_STATE_CRANKING) {
        // Came from cranking event, print the stats.
        foc_math_print_speed_stats(motor, false);
    }
    return WRSM_SUPER_STATE_STOPPING;
}

// --- STATE: ENGINE_STOPPING (COAST-DOWN TO STANDBY) ---
static void state_stopping_entry(motor_all_state_t *motor) {
    motor->m_field_override_active = true; // goal is to stop, take field control back.
    motor->m_field_override_current = 0.0f;
    mcpwm_foc_set_current(0.0f); // make motor freewheel without killing fet diodes.
}

static wrsm_super_state_t state_stopping_tick(motor_all_state_t *motor, float dt) {
    // If stopping occurred during Open-Loop (standstill / low speed start),
    // float the gates immediately since there is no high-speed BEMF!
    if (motor->m_phase_observer_override) {
        mcpwm_foc_stop_pwm(false); // Sets m_state = MC_STATE_OFF & clears override
        return WRSM_SUPER_STATE_OFF;
    }
    // watch for FOC Control to transistion to MC_STATE_OFF/CONTROL_MODE_NONE at minimum ERPM
    if (motor->m_state == MC_STATE_OFF) {
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
    if (motor->m_state != MC_STATE_OFF) {
        return WRSM_SUPER_STATE_FAULT;
    }
    // RPM Guard: Prevent entering PRE_EXCITE if the MGU is already spinning
    float current_erpm = fabsf(mcpwm_foc_get_rpm());
    float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * POLE_PAIRS;
    
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
    motor->m_field_override_active = true;
    motor->m_field_override_current = 2.5f;

    float target_erpm   = motor->m_conf->wrsm_crank_target_rpm;
    float ramp_time_sec = motor->m_conf->wrsm_crank_ramp_time;
    float handoff_erpm  = motor->m_conf->foc_openloop_rpm;
    float target_iq     = motor->m_conf->wrsm_crank_target_iq;

    float kp = motor->m_conf->s_pid_kp;
    float max_current = motor->m_conf->l_current_max;

    // Calculate starting speed offset for target_iq amp target current
    float speed_error_offset = target_iq / (kp * 0.05f * max_current);

    // Calculate the parallel ramp slope
    float ramp_slope = (target_erpm - speed_error_offset) / ramp_time_sec;
    motor->m_conf->s_pid_ramp_erpms_s = ramp_slope;

    // Cleanly configure open-loop driver parameters in RAM
    motor->m_conf->foc_sl_openloop_time_lock = 0.0f;
    motor->m_conf->foc_sl_openloop_time_ramp = handoff_erpm / ramp_slope;
    motor->m_conf->foc_sl_openloop_time      = 0.0f;
    motor->m_speed_pid_set_rpm               = speed_error_offset;

    // Start the stats
    foc_math_clear_speed_stats(motor);
    // Command the speed PID loop to execute
    mcpwm_foc_set_pid_speed(target_erpm);
}

static wrsm_super_state_t state_cranking_tick(motor_all_state_t *motor, float dt) {
    float current_mgu_erpm = fabsf(mcpwm_foc_get_rpm());
    float started_threshold_erpm = motor->m_conf->wrsm_crank_target_rpm * 1.5f; //150% of start erpm

    if (current_mgu_erpm > started_threshold_erpm) { // Already beyond 150% target_erpm, must be driven by engine.
        return WRSM_SUPER_STATE_ALTERNATOR;
    } else if (state_timer >= CRANK_TIMEOUT_SEC) { // Max start time elapsed.
        return WRSM_SUPER_STATE_STOPPING;
    }
    return WRSM_SUPER_STATE_CRANKING;
}

static wrsm_super_state_t state_alternator_guard(motor_all_state_t *motor, wrsm_super_state_t from_state) {
    if (from_state == WRSM_SUPER_STATE_CRANKING) {
        // Came from cranking event, print the stats.
        foc_math_print_speed_stats(motor, true);
    }
    return WRSM_SUPER_STATE_ALTERNATOR;
}

// --- STATE: ALTERNATOR_ACTIVE ---
static void state_alternator_entry(motor_all_state_t *motor) {
    motor->m_field_override_active = false; // Relinquish manual control to optimal loss curves
}

static wrsm_super_state_t state_alternator_tick(motor_all_state_t *motor, float dt) {
    // check for rpm drop so low that stator has turned off unexpectedly.
    if (motor->m_state == MC_STATE_OFF) {
        //return WRSM_SUPER_STATE_FAULT;
        return WRSM_SUPER_STATE_OFF;
    }

    /*
     * JAH FUTURE ROADMAP: ALTERNATOR CHARGING REGULATION & STALL DETECTION
     *
     * 1. Dual-Mode Charging Loop:
     *    - Primary Mode (CAN-Bus Directed):
     *      Regulate output stator current (Iq) using target charging voltages 
     *      broadcasted by the STM32 BMS over CAN. Override and throttle Iq 
     *      instantly if the BMS reports battery charging current exceeds limits.
     *    - Monitor CAN-Bus for additional parameters for performance gain, i.e.
     *      set charging current to a low/zero value on WOT request, possibly with
     *      additional charge current under braking to make up for the missing charge.
     *    - Backup Mode (Local Fallback):
     *      Run a local down-counter millisecond timer. If no CAN-bus packet 
     *      is received from the BMS within the timeout window (e.g., 250ms), 
     *      seamlessly fall back to local voltage regulation using local V_batt 
     *      against 'wrsm_alt_target_voltage'. Ramping logic should prevent 
     *      sudden current steps during fall-back/recovery.
     *
     * 2. Stall-Catch Detection via Acceleration (d_speed/dt):
     *      Rather than waiting for absolute speed to sag below critical limits 
     *      (a lagging indicator), monitor the filtered first derivative of speed 
     *      ('motor->m_speed_deriv_filtered'). If MGU RPM is below the idle threshold 
     *      AND deceleration exceeds 'wrsm_stall_decel_trigger' (e.g., -800 ERPM/s^2) 
     *      due to a heavy transient load step, instantly command a state transition 
     *      to STALL_CATCH to wake up the stator motoring mode.
     *
     * 3. Idle and slightly below Idle Current reduction:
     *      From the minimum Idle ERPM to the top of the stall catch ERPM threshold,
     *      ramp the current production to zero over this window (stall catch ERPM
     *      threshold = 0% requested current -> min Idle ERPM 100% requested current).
     */
    
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
    /*
     * JAH FUTURE ROADMAP: ANTI-STALL FLYING CATCH CONTROLLER
     *
     * 1. Transition to Stiff Motoring Loop:
     *    - Instantly override the FOC stator current limits to maximum motoring torque 
     *      ('wrsm_stall_catch_max_iq') and disable generator behavior.
     *    - Swap the standard speed PID gains to our dedicated anti-stall parameters 
     *      ('wrsm_stall_catch_kp', 'wrsm_stall_catch_ki') to create a fast, stiff 
     *      motoring response that fights crankshaft deceleration.
     *    - Command the Speed PID targeting 'wrsm_stall_catch_target_rpm' 
     *      (MGU target idle, e.g., 2400 ERPM).
     *
     * 2. Exit/Recovery Routing:
     *    - Recovery Success: If engine RPM climbs back above 'MGU_RPM_RUNNING_THRESHOLD',
     *      relinquish motoring control, restore regular FOC configurations, 
     *      and transition back to ALTERNATOR_ACTIVE mode.
     *    - Recovery Failure/Engine Stalled: If engine speed falls below the absolute 
     *      stationary limit ('MGU_RPM_STOPPED_THRESHOLD') OR if 'STALL_RECOVERY_TIMEOUT' 
     *      (1.5s) is reached without recovery, float the gates and return to STANDBY_OFF 
     *      to prevent burning stator windings on a locked engine.
     */


    // JAHTODO add actual code for stall catch
    // set stall catch speed pid accel rate, should be higher than start, or even off.
    // switch to mcpwm_foc_set_pid_speed(calc target)
    
    float current_mgu_erpm = fabsf(mcpwm_foc_get_rpm());

    if (current_mgu_erpm > MGU_RPM_RUNNING_THRESHOLD) {
        return WRSM_SUPER_STATE_ALTERNATOR;
    } else if (current_mgu_erpm < MGU_RPM_STOPPED_THRESHOLD || state_timer >= STALL_RECOVERY_TIMEOUT) {
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
        float erpm_threshold = MGU_RPM_STOPPED_THRESHOLD * POLE_PAIRS;
        
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
        .guard = state_stopping_guard,
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
        .guard = state_alternator_guard,
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
}

// ============================================================================
// JAH: Isolated Volatile Telemetry Flags & APIs
// ============================================================================
static volatile bool m_telemetry_enabled = false;

void wrsm_supervisor_set_telemetry_enabled(bool enabled) {
    m_telemetry_enabled = enabled;
}

bool wrsm_supervisor_get_telemetry_enabled(void) {
    return m_telemetry_enabled;
}

void wrsm_supervisor_update(motor_all_state_t *motor, float dt) {
    m_active_motor = motor;
    state_timer += dt;
    float current_speed = mcpwm_foc_get_rpm();
    float raw_accel = 0.0f;

    if (dt > 0.0f) {
        raw_accel = (current_speed - motor->m_speed_prev_for_accel) / dt;
    }
    motor->m_speed_prev_for_accel = current_speed;
    motor->m_accel = raw_accel;

    // Apply native exponential low-pass filter
    float filter_coef = motor->m_conf->wrsm_accel_filter_coef;
    UTILS_LP_FAST(motor->m_accel_filtered, raw_accel, filter_coef);

    // ============================================================================
    // JAH: Web Serial Diagnostic Telemetry (1 kHz Execution)
    // ============================================================================
    
    // Persistent EMA variables (Zero memory footprint, zero arrays)
    static float accel_avg = 0.0f;
    static float accel_mad = 0.0f;
    static float iq_avg = 0.0f;
    static float iq_mad = 0.0f;
    static float iq_target_avg = 0.0f;
    static float id_avg = 0.0f;
    static float if_avg = 0.0f;
    static float if_mad = 0.0f;
    static float fpwm_avg = 0.0f;
    static float fpwm_mad = 0.0f;
    static float vbus_avg = 0.0f;
    static float duty_avg = 0.0f;
    static float stator_fw_avg = 0.0f;

    // Block Transient Detectors (Reset every 20ms telemetry window)
    static float id_min = 999.0f;
    static float id_max = -999.0f;
    static float vbus_min = 999.0f;
    static float vbus_max = -999.0f;
    static float duty_max = -999.0f;
    static float if_min = 999.0f;
    
    static int telemetry_divider = 0;

    // 1. Fetch live raw high-speed variables from physical FOC registers
    float raw_rpm   = mcpwm_foc_get_rpm();
    float raw_iq    = mcpwm_foc_get_iq();
    float raw_iq_tgt= mcpwm_foc_get_iq_set();
    float raw_id    = mcpwm_foc_get_id();
    float raw_if    = motor->m_field_current;
    float raw_fpwm  = motor->m_field_duty;
    float raw_vbus  = motor->m_motor_state.v_bus;
    float raw_duty  = mcpwm_foc_get_duty_cycle_now();
    float raw_sfw   = motor->m_i_fw_set;

    // Continuous Double-EMA Filters (alpha = 0.10f rough equivalent to a 10ms-15ms time-constant)
    const float alpha = 0.10f;
    
    // Speed Derivatives & Vibrations
    accel_avg  += alpha * (motor->m_accel - accel_avg);
    accel_mad  += alpha * (fabsf(motor->m_accel - accel_avg) - accel_mad);

    // Stator Torque Currents
    iq_avg     += alpha * (raw_iq - iq_avg);
    iq_mad     += alpha * (fabsf(raw_iq - iq_avg) - iq_mad);
    iq_target_avg += alpha * (raw_iq_tgt - iq_target_avg);

    // Magnetizing & Rotor Excitation
    id_avg     += alpha * (raw_id - id_avg);
    if_avg     += alpha * (raw_if - if_avg);
    if_mad     += alpha * (fabsf(raw_if - if_avg) - if_mad);

    // Field H-Bridge Duty Cycle Chatter
    fpwm_avg   += alpha * (raw_fpwm - fpwm_avg);
    fpwm_mad   += alpha * (fabsf(raw_fpwm - fpwm_avg) - fpwm_mad);

    // Voltage & Saturation Limits
    vbus_avg   += alpha * (raw_vbus - vbus_avg);
    duty_avg   += alpha * (raw_duty - duty_avg);
    stator_fw_avg += alpha * (raw_sfw - stator_fw_avg);

    // 3. Update Block-Level Transient Peak Catchers
    if (raw_id > id_max)     id_max = raw_id;
    if (raw_id < id_min)     id_min = raw_id;
    if (raw_vbus > vbus_max) vbus_max = raw_vbus;
    if (raw_vbus < vbus_min) vbus_min = raw_vbus;
    if (raw_duty > duty_max) duty_max = raw_duty;
    if (raw_if < if_min)     if_min = raw_if;

    // 4. Downsample Transmission Rate to 50 Hz (Executes every 20ms)
    telemetry_divider++;
    if (telemetry_divider >= 20) {
        telemetry_divider = 0;

        if (m_telemetry_enabled) {
            // Instantiate and pack our telemetry packet struct
            wrsm_telemetry_packet_t packet;
            packet.start_marker  = 0xAA;
            packet.super_state   = (uint8_t)current_state;
            packet.mc_state      = (uint8_t)motor->m_state;
            packet.ctrl_mode     = (uint8_t)motor->m_control_mode;
            packet.rpm           = raw_rpm;
            packet.accel_avg     = accel_avg;
            packet.accel_mad     = accel_mad;
            packet.iq_avg        = iq_avg;
            packet.iq_mad        = iq_mad;
            packet.iq_target     = iq_target_avg;
            packet.id_avg        = id_avg;
            packet.id_min        = id_min;
            packet.id_max        = id_max;
            packet.if_avg        = if_avg;
            packet.if_mad        = if_mad;
            packet.field_pwm_avg = fpwm_avg;
            packet.field_pwm_mad = fpwm_mad;
            packet.vbus_avg      = vbus_avg;
            packet.vbus_min      = vbus_min;
            packet.vbus_max      = vbus_max;
            packet.duty_avg      = duty_avg;
            packet.duty_max      = duty_max;
            packet.if_min        = if_min;
            packet.stator_fw_id  = stator_fw_avg;

            // Generate the XOR Packet Checksum
            uint8_t calc_checksum = 0;
            uint8_t *packet_bytes = (uint8_t*)&packet;
            for (int i = 0; i < sizeof(wrsm_telemetry_packet_t) - 1; i++) {
                calc_checksum ^= packet_bytes[i];
            }
            packet.checksum = calc_checksum;

            // Null-Byte Safe Character Stream Output
            for (int i = 0; i < sizeof(wrsm_telemetry_packet_t); i++) {
                commands_printf("%c", packet_bytes[i]);
            }
        }

        // 5. Reset the Peak Catchers for the next 20ms block
        id_min   = 999.0f;
        id_max   = -999.0f;
        vbus_min = 999.0f;
        vbus_max = -999.0f;
        duty_max = -999.0f;
        if_min   = 999.0f;
    }

    // --- GLOBAL SAFETY TRANSITION INTERLOCKS ---
    // 1. Terminal Hardware ESTOP Check (Absolute dead-end)
    if (motor->m_field_ESTOP_LOCKOUT) {
        requested_state = WRSM_SUPER_STATE_ESTOP;
        process_internal_transition(motor);
        return;
    }

    // 2. Recoverable Inverter/Stator Fault Check (Normal recoverable fault)
    if (mc_interface_get_fault() != FAULT_CODE_NONE && current_state != WRSM_SUPER_STATE_ESTOP) {
        requested_state = WRSM_SUPER_STATE_FAULT;
        process_internal_transition(motor);
        return;
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
    wrsm_super_state_t resolved_state = new_requested_state;
    if (state_table[new_requested_state].guard) {
        resolved_state = state_table[new_requested_state].guard(motor, current_state);
    }

    // If the guard decided to stay where we are (resolved == current), abort transition
    if (resolved_state == current_state) {
        return;
    }

    // Execute state entry action EXACTLY ONCE
    if (state_table[resolved_state].on_entry) {
        state_table[resolved_state].on_entry(motor);
    }

    // Accept transition and clear local timer
    current_state = resolved_state;
    requested_state = resolved_state;
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

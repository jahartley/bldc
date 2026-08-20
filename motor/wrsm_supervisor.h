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

// Web logging telemetry data pack
#pragma pack(push, 1)
typedef struct {
    uint8_t start_marker;     // 0xAA - Fixed alignment byte for browser synchronization
    uint32_t packet_id;       // 4-byte sequence counter
    uint8_t super_state;      // WRSM Supervisor State ID (Enum)
    uint8_t mc_state;         // VESC Motor State (Enum)
    uint8_t ctrl_mode;        // FOC Active Control Mode (Enum)
    float   rpm;              // Decimated: Instantaneous Speed (ERPM)
    float   accel_avg;        // Double-EMA: Smoothed Acceleration (ERPM/s)
    float   accel_mad;        // Double-EMA: Torsional Crankshaft Vibration Width
    float   iq_avg;           // Double-EMA: Stator Torque-producing Current (A)
    float   iq_mad;           // Double-EMA: Stator Current Noise/Hunting Tracker (A)
    float   iq_target;        // Double-EMA: Demanded Torque Current Target (A)
    float   id_avg;           // Double-EMA: Stator Demagnetizing Current (A)
    float   id_min;           // Block-Min: Peak Stator Demagnetizing Current in 20ms block
    float   id_max;           // Block-Max: Peak Magnetizing Current in 20ms block
    float   if_avg;           // Double-EMA: Measured Winding Excitation Current (A)
    float   if_mad;           // Double-EMA: Rotor Current Ripple / PI Loop Hunting (A)
    float   field_pwm_avg;    // Double-EMA: Rotor H-Bridge Duty Cycle Applied (0.0 to 1.0)
    float   field_pwm_mad;    // Double-EMA: Rotor H-Bridge Duty Cycle Chatter Width
    float   vbus_avg;         // Double-EMA: Battery Bus Voltage (V)
    float   vbus_min;         // Block-Min: Battery Cranking Voltage Sag (V)
    float   vbus_max;         // Block-Max: Alternator Load-Dump Spike Tracker (V)
    float   duty_avg;         // Double-EMA: Stator Inverter Duty Cycle Average (0.0 to 1.0)
    float   duty_max;         // Block-Max: Inverter Voltage-Saturation/Clipping Tracker (0.0 to 1.0)
    float   if_min;           // Block-Min: REPLACED lambda_min -> Peak Negative Rotor Current Sag
    float   stator_fw_id;     // Double-EMA: Stator-assisted Field Weakening Target (A)
    uint8_t checksum;         // XOR checksum of all previous 84 bytes
} wrsm_telemetry_packet_t;
#pragma pack(pop)

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
void wrsm_supervisor_request_state(wrsm_super_state_t requested_state);

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

// JAH: WRSM 50Hz Binary Telemetry Control APIs
void wrsm_supervisor_set_telemetry_enabled(bool enabled);
bool wrsm_supervisor_get_telemetry_enabled(void);

#endif /* WRSM_SUPERVISOR_H_ */
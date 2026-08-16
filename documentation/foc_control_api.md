# Quick Reference: Common VESC States & Modes

## Standard VESC enum definitions

### mc_state (Physical Driver State) motor->m_state
- MC_STATE_OFF: Stator PWM switching is disabled. The phases are completely floated (high-impedance)
- MC_STATE_RUNNING: The stator PWM is active and driving current or voltage
- MC_STATE_FAULT: A physical stator fault has been triggered, disabling the gates

### mc_control_mode (Stator Control Loop) motor->m_control_mode
- CONTROL_MODE_NONE: No active loop. The driver is unmanaged
- CONTROL_MODE_CURRENT: Torque-producing Q-axis current control
- CONTROL_MODE_SPEED: Closed-loop Speed PID (ERPM) tracking
- CONTROL_MODE_CURRENT_BRAKE: Controlled regenerative braking
- CONTROL_MODE_DUTY: Direct voltage duty cycle modulation
- CONTROL_MODE_OPENLOOP: Speed-stepped open-loop vector rotation (used primarily during sensorless starting)

## Core State & Control Mode Getters
Use these functions to query what the physical power stage and FOC controllers are currently executing under the hood:
### mcpwm_foc_get_state(void)
Returns: mc_state (enumeration of active hardware states)
- MC_STATE_OFF (0): Gates floated. Motor is in high-impedance (freewheel)
- MC_STATE_RUNNING (1): Inverter is actively switching
- MC_STATE_FAULT (2): Stator driver faulted out (disabled outputs).

### mcpwm_foc_control_mode(void)
Returns: mc_control_mode (active stator closed-loop configuration)
- CONTROL_MODE_NONE (0): No active loop
- CONTROL_MODE_CURRENT (1): closed-loop current/torque control
- CONTROL_MODE_SPEED (2): closed-loop Speed PID tracking
- CONTROL_MODE_CURRENT_BRAKE (3): Regenerative braking current control
- CONTROL_MODE_DUTY (4): Direct duty-cycle voltage drive
- CONTROL_MODE_OPENLOOP (8): Open-loop synchronous vector spinning

## Stator Control Commands
Use these functions to command the stator power stage:
### mcpwm_foc_stop_pwm(bool is_second_motor)
Purpose: Instantly disables and floats the stator phase gates (CONTROL_MODE_NONE & MC_STATE_OFF)

### mcpwm_foc_set_current(float current)
Purpose: Sets closed-loop stator Q-axis torque current (m_iq_set) and forces CONTROL_MODE_CURRENT
- Passing 0.0f active-clamps torque while keeping the gates switching and sensorless observer running

### mcpwm_foc_set_pid_speed(float rpm)
Purpose: Sets the target Electrical RPM (ERPM) for the speed PID controller (CONTROL_MODE_SPEED)

### mcpwm_foc_set_brake_current(float current)
Purpose: Sets a target braking current, engaging CONTROL_MODE_CURRENT_BRAKE
- This forces the torque vector to dynamically fight the current direction of rotation

### mcpwm_foc_set_openloop_current(float current, float rpm)
Purpose: Forces open-loop current vector rotation at a designated electrical speed
- Used by VESC for stationary/starting assist prior to observer convergence

## Sensorless Observer & Telemetry Getters
Use these APIs to poll real-time physical telemetry estimated by the closed-loop tracking algorithms:

### mcpwm_foc_get_rpm(void)
Returns: Electrical RPM (PLL-filtered and smooth)
- Divide by pole pairs to get physical shaft speed

### mcpwm_foc_get_rpm_fast(void) / mcpwm_foc_get_rpm_faster(void)
Returns: Raw, unfiltered electrical RPM estimations (faster response, noisier)

### mcpwm_foc_get_phase_observer(void)
Returns: The current estimated electrical angle of the rotor in degrees [0.0, 360.0]

### mcpwm_foc_get_id(void) / mcpwm_foc_get_iq(void)
Returns: Real-time measured physical D-axis (flux-producing) and Q-axis (torque-producing) currents in Amperes

### mcpwm_foc_get_id_set(void) / mcpwm_foc_get_iq_set(void)
Returns: The active target currents commanded by the FOC loop

### mcpwm_foc_get_est_lambda(void)
Returns: The live estimated flux linkage (Webers)
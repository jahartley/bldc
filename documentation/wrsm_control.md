# WRSM Control
## Introduction
This fork is designed to control a wound rotor synchronous motor, in my case a 36V rated MGU430-003 from a 2009 Saturn Vue Hybrid. This MGU is mostly a standard car alternator, wound rotor claw pole design, but with extra small permanent magnets in between the claw poles, which provide better field shape and a small amount of flux linkage at zero amps field current. Because we get to control the flux linkage, via field current, we can directly change field strength and can weaken the field without using Id current.

This MGU is going to be used at 12V to start a car engine via the belt (as originally designed, just at lower voltage), then transition to alternator mode to supply current for all car loads and battery charging.

## Test Hardware
My test hardware is a VESC_LABS classic plus, hooked to 2x parallel car batteries. I have added an H Bridge (dual BTS7960) board to control the field current and provide rapid field collapse in an emergency stop situation and an ACS712-05 for current monitoring and control. I am using the servo/ppm pin to drive on half of the H bridge board with 5kHz PWM, and the comm header UART TX pin for H bridge enable. Comm header EXT_ADC is reading the ACS712-05.

## MGU430-003 motor detection

Motor Has 8 pole pairs (ERPM/8 = MGU RPM), 48 stator slots, 118mm rotor OD. 118.4mm Stator ID. Field Coil R 5.6 ohms.

Field_Current_A,Field_Voltage_V,Motor_Current_A,Motor_R_mOhm,Motor_L_uH,Motor_ld_lq_diff,Flux_Linkage_mWb,Flux_Linkage_Wb
0.00,0.00,152.02,11.50,38.10,15.94,3.08,0.00308
0.50,2.88,150.86,11.70,37.99,16.27,6.80,0.00680
1.00,6.00,151.75,11.58,35.25,15.82,10.08,0.01008
2.00,11.22,151.79,11.60,22.12,12.06,12.41,0.01241
2.92,17.00,150.91,11.70,19.22,10.26,13.20,0.01320

ERPM testing. Unloaded motor was set to 100% duty (95% is max achieved) at 14.2V V_batt, results:

Field_Current_A,Max_ERPM,Bus_Voltage_V
0.60,12050,14.2
0.80,9300,14.2
1.00,7900,14.2
1.50,6540,14.2

Drill spin testing:

Drill_Speed,Field_Current_A,Field_Voltage_V,Electrical_Frequency_Hz,Line_to_Line_Vrms,Line_to_Line_Vpp,Average_DC_offset,Estimated_ERPM,Estimated_MGU_RPM,Estimated_K_e_ERPM,Estimated_K_e_MGU_RPM,Estimated_peak_phase_to_neutral_V,Field_Power_W
Speed 1,0,0,84,1.62,5,,5040,630,0.321428571,2.571428571,1.322724461,0
Speed 2,0,0,253,4.8,14.2,0.181,15180,1897.5,0.316205534,2.529644269,3.919183588,0
Speed 1,0.5,2.88,83.3,3.41,12.4,0.254,4998,624.75,0.682272909,5.458183273,2.784253341,1.44
Speed 1,1.07,6,80.6,5.96,19.2,0.263,4836,604.5,1.23242349,9.859387924,4.866319622,6.42
Speed 1,2,11.22,80.6,7.67,24,0.276,4836,604.5,1.586021505,12.68817204,6.262528776,22.44
Speed 1,3,16.88,80.6,8.04,25.8,0.267,4836,604.5,1.662531017,13.30024814,6.564632511,50.64



## Control Strategy
I want to fully integrate field control into the FOC system. By using my bench supply to set specific field currents, and running the VESC tool motor detection, I have built a look up table for Motor L, Motor Ld-lq diff, and Motor Flux Linkage. Looking at the calculations performed at normal FOC init, where the derivative values are calculated, I may either precompute additional look up tables or look up then compute the derivative values.

Values requiring update list:
// --- JAH ADDED INJECTED LOOKUP VARIABLES FOR WRSM DYNAMIC FOC ---
	float m_injected_flux;         			foc_motor_flux_linkage	// Live stator flux linkage (lambda) from initial table of vesc motor testing
	float m_injected_ld;           			p_ld					// Live d-axis inductance (Ld)
	float m_injected_lq;           			p_lq					// Live q-axis inductance (Lq)
	float m_injected_l;            			foc_motor_l				// Live stator inductance (Ld+Lq)/2 from initial table of vesc motor testing
	float m_injected_ld_lq_diff;   			foc_motor_ld_lq_diff	// Live saliency indicator (Lq - Ld) from initial table of vesc motor testing
	float m_injected_inv_ld;       									// Live inverse d-axis inductance (1 / Ld)
	float m_injected_inv_lq;       									// Live inverse q-axis inductance (1 / Lq)
	float m_injected_p_inv_ld_lq;  			p_inv_ld_lq				// Live inverse saliency difference (1/Lq - 1/Ld)
	float m_injected_p_v2_v3_inv_avg_half; 	p_v2_v3_inv_avg_half 	// Live HFI average of inverse d and q axis inductances: 0.5 * (1/Ld + 1/Lq)

### Current modes/operating plan
1. Start mode. If the MGU is not spinning, and either a press and hold start button or possible future can bus command, will put us into start mode, where we will go to speed pid control, initially with the speed set to 200 engine rpm (must calc engine -> MGU via real belt ratio * 8 to get ERPM value for speed pid) and a testing determined open loop current. allow braking will be false, so that when the engine starts the MGU free wheels up with the engine, and expected low idle is 600 engine RPM. When the button is released, there are two choices, if erpm is above the speed set point, we know the engine is running, and will go to alternator mode. If the speed is near the set speed, we will freewheel down to a stop.

2. Alternator mode. If the system boots with the MGU already spinning, or we get here from other modes, we will use a PID loop to monitor and adjust current output to maintain V_batt at a specific value. The MGU will be put into current control mode, with target i_q set and controlled by this loop.

3. Stall Catch mode. The overall supervisor function that sets these modes will watch MGU rpm for a rapid fall in rpm, while operating at less than 1000 engine RPM, in alternator mode. If this happens, the supervisor will switch the MGU to speed pid mode, with a speed target of 500 or 600 engine rpm (just below the lowest target engine idle speed) and let the MGU assist in catching the engine to prevent a stall. When the speed pid target is reached or exceeded again, we move back to alternator mode. We may want different speed pid P,I,D values vs start mode.

### Control strategy levels
1. Continuously read actual field current, look up or calculate all parameters that vary at different field currents, update them all, so that the continuously correct values are used everywhere. This runs independently of trying to control the field current.
2. Functions to get/set field H bridge enable pin, pwm pin free running hardware pwm.
3. Field current PID and target current calculation. This level uses a PID loop to try and follow a target field current, which is either overridden and set by a higher-level function, or is calculated here based on the current stator i_q target current to provide best MGU efficiency.
4. Overall Supervisor. This level does the high-level state machine, decides which state to be in and when to transition based on MGU parameters and/or outside inputs from buttons or can bus. This level sets FOC modes, decides field i_f current overrides if needed, and performs a very slow alternator charging voltage -> required current PID loop.

## Safety, faults, EMERGENCY STOP
This is a race car automotive application, and one required item is a emergency stop battery disconnect switch. When hit by the driver or track worker, this will immediately disconnect the battery from the entire electrical system to include starters and alternator/generators. I plan on connecting the field current H Bridge B+ source to a throw in the relay that trips the emergency stop disconnect, wired so that when the estop is triggered, the field current H bridge B+ terminal is connected to ground. This ensures that collapsing field magnetic field does not power up the cars now disconnected electrical system. VESC full_brake_hw stop function also connects all motor phases to ground to prevent any current or voltage from the potentially still spinning MGU (which has a field even at zero amps) from powering the disconnected electrical system as well. 

### Normal faults
Normal faults that are not EMERGENCY STOP, should be designed around the fact that this MGU, while not a BLDC machine, still has a significant amount of flux with zero field current, and may not be able to stop spinning due to its connection to a comparatively large engine. An in-depth review of all the faults, and if they rate a system stop will need to be completed before use outside of a test environment.

## EQUATION/FUNCTION Review.
I need to review the operation of: MTPA Maximum torque per amp algorithm, FW Field weakening I need to review this more for my motor type, this is trying to add Id normally, I probably should reduce field current If instead. 

## Application specific defaults.
Due to the unique nature of my application and the specific safety concerns, I will be hard coding some default configuration values that would not normally be put into the code base. Identified specific defaults listed below:

MCCONF_FOC_OBSERVER_TYPE        3 // 3 = FOC_OBSERVER_MXLEMMING_LAMBDA_COMP
MCCONF_FOC_SAT_COMP_MODE        0 // 0 = SAT_COMP_DISABLED
MCCONF_FOC_MTPA_MODE            1 // 1 = MTPA_MODE_IQ_TARGET
MCCONF_FOC_CC_DECOUPLING        3 // 3 = FOC_CC_DECOUPLING_CROSS_BEMF
MCCONF_S_PID_MIN_ERPM           0.0f
MCCONF_S_PID_ALLOW_BRAKING      false 

## Variable Poisoning
To ensure that I am not overlooking the use of either a real time updated variable, or a pin change, I have added #define statements so that I will have build errors if I don’t fix them in code. Future versions may try to leave more of the original code base alone and properly update by including extra files, but in this test version I will be replacing many variable names.

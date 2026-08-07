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



## Control Strategy


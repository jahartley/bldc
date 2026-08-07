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
I want to fully integrate field control into the FOC system. By using my bench supply to set specific field currents, and running the VESC tool motor detection, I have built a look up table for Motor L, Motor Ld-lq diff, and Motor Flux Linkage. Looking at the calculations performed at normal FOC init, where the derivative values are calculated, I may either precompute additional look up tables, or lookup then compute the derivative values.

- I_f ADC. When the 20-30kHz ADC DMA runs for the group containing the EXT_ADC pin, we should do a 2 or 3 sample moving average filter on the adc value, convert to current, lookup and update our values in the motor state struct.

Values requiring update list:
Group 1: The Raw Lookups (Direct from your 1kHz Table)
These three variables must be directly read and linearly interpolated from your bench data based on your field current (If):
•	flux_linkage (lambda): The total magnetic flux linking the stator phases, which represents the combined strength of the permanent magnets and the field coil.
•	Motor_L (L_base): The baseline or average stator inductance, mathematically representing (Ld + Lq) / 2.
•	Motor_ld_lq_diff (Delta L): The saliency indicator, which is the physical difference between the q-axis and d-axis inductances (Lq - Ld).
Group 2: The Derived Core FOC Variables (Recomputed from Lookups)
Your code must immediately calculate these explicit axis inductances right after updating the raw lookups:
•	Ld (d-axis inductance): Calculated as: Motor_L minus (Motor_ld_lq_diff / 2). It dictates the motor's behavior along the magnetic flux axis.
•	Lq (q-axis inductance): Calculated as: Motor_L plus (Motor_ld_lq_diff / 2). It dictates the motor's behavior along the torque-producing axis.
Group 3: Real-Time Controller Gain Adaptations (For Current Loop Stability)
Because physical inductance drops by half at high field current, you must scale your Proportional gains to prevent current loop instability:
•	Current_Loop_Kp_d: Calculated as: Ld multiplied by your desired current loop bandwidth in radians per second.
•	Current_Loop_Kp_q: Calculated as: Lq multiplied by your desired current loop bandwidth in radians per second.
Group 4: Voltage Decoupling Terms (Cross-Coupling Compensation)
These are feedforward terms calculated during the high-speed voltage generation phase to keep the d and q axes from corrupting each other at high speeds:
•	Vd_decouple: Calculated as: negative electrical speed (omega_e) multiplied by Lq multiplied by Iq.
•	Vq_decouple: Calculated as: electrical speed (omega_e) multiplied by the quantity (Ld multiplied by Id plus flux_linkage).


EQUATION/FUNCTION Review.
I need to review the operation of: MTPA Maximum torque per amp algorithm, FW Field weakening (I need to review this more for my motor type, this is trying to add Id normally, I probably should reduce field current If instead. 


## 20kHz I_f lookup strategy
This is my rough draft for the 20kHz current lookup piecewise value interpolation optimization. A 2 or 3 sample moving average filter then adc -> I_f computation should be performed before this lookup updates the motor state struct values.

```c
#include <stdint.h>
#include <stdbool.h>

// 1. Structure to hold precalculated parameters for each current sector
typedef struct {
    float lower_bound_if; // The If boundary value for this row (e.g., 2.00, 1.00, 0.50, 0.00)
    
    // Base values at this exact lower bound current
    float base_flux;
    float base_ld;
    float base_lq;
    float base_l_avg;
    float base_ld_lq_diff;
    
    // Slopes precalculated on PC (Delta_Y / Delta_X)
    // Safe for both positive and negative slopes
    float slope_flux;
    float slope_ld;
    float slope_lq;
    float slope_l_avg;
    float slope_ld_lq_diff;
} if_lookup_row_t;

// 2. Define the lookup table array (5 points = 4 interpolation sectors)
// Arranged highest-to-lowest to optimize for high-load execution speed
#define LOOKUP_SECTORS 4
static const if_lookup_row_t if_table[LOOKUP_SECTORS] = {
    // Row 0: Handling the 2.00A to 2.92A sector
    { .lower_bound_if = 2.00f, .base_flux = 12.41f, .slope_flux = 0.86f, /* ... other bases & slopes */ },
    // Row 1: Handling the 1.00A to 2.00A sector
    { .lower_bound_if = 1.00f, .base_flux = 10.08f, .slope_flux = 2.33f, /* ... other bases & slopes */ },
    // Row 2: Handling the 0.50A to 1.00A sector
    { .lower_bound_if = 0.50f, .base_flux = 6.80f,  .slope_flux = 6.56f,  /* ... other bases & slopes */ },
    // Row 3: Fallback boundary for the 0.00A to 0.50A sector
    { .lower_bound_if = 0.00f, .base_flux = 3.08f,  .slope_flux = 7.44f,  /* ... other bases & slopes */ }
};

// 3. Fast Parameter Injection Function (Executes at 20kHz - 30kHz inside FOC Loop)
void update_hybrid_motor_parameters(float filtered_if, motor_foc_state_t *foc_state) {
    int target_row = LOOKUP_SECTORS - 1; // Default fallback to the lowest sector (Row 3, 0.00A)

    // Sequential search from highest to lowest. 
    // Breaks instantly on the first true condition, optimizing for high-load states.
    for (int i = 0; i < LOOKUP_SECTORS - 1; i++) {
        if (if_table[i].lower_bound_if <= filtered_if) {
            target_row = i;
            break;
        }
    }

    // Reference the matched row pointer directly to avoid array lookup overhead
    const if_lookup_row_t *row = &if_table[target_row];

    // Single Delta Optimization: Calculate the delta current exactly once
    float shared_delta = filtered_if - row->lower_bound_if;

    // Zero-guard boundary: Handle minor negative ADC tracking noise safely
    if (shared_delta < 0.0f) {
        shared_delta = 0.0f;
    }

    // Multi-variable single-cycle execution phase. 
    // Uses 1-cycle multiplies and 1-cycle additions. No runtime divisions or conditionals.
    foc_state->flux_linkage    = row->base_flux        + (shared_delta * row->slope_flux);
    foc_state->Ld              = row->base_ld          + (shared_delta * row->slope_ld);
    foc_state->Lq              = row->base_lq          + (shared_delta * row->slope_lq);
    foc_state->motor_l         = row->base_l_avg       + (shared_delta * row->slope_l_avg);
    foc_state->motor_ld_lq_diff = row->base_ld_lq_diff + (shared_delta * row->slope_ld_lq_diff);
}
```
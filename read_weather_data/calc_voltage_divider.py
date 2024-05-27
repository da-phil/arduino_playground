#!/usr/bin/env python3

VOLT_DIV_OUT_IMPEDANCE = 100e3  # approximate output impedance for voltage divider
RL = 250e3  # ADC input impedance
R1 = 68e3

V_ADC_MAX = 3.3
V_IN_MAX = 14.64  # open-loop voltage of PV panel
FACTOR = V_ADC_MAX / V_IN_MAX
R2L = FACTOR * R1  # unloaded voltage divider case
# determine R2 from given factor:
R2 = (R2L * RL) / (RL - R2L)

print(
    f"For given factor {FACTOR:.5f} to scale input voltage {V_IN_MAX} to ADC ref voltage {V_ADC_MAX} => R2={R2:.2f}"
)


## Example: given choice for R2
R2 = 19.5e3
# R2 when loaded with ADC input impedance (loaded voltage divider case)
R2L = (R2 * RL) / (R2 + RL)
FACTOR = R2L / (R1 + R2L)

print(f"Voltage divider factor for given R2({R2}): {FACTOR:.5f}")

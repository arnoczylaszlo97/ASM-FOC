#ASM-FOC
Torque and speed control of induction machine with three phase two level inverter.
Created in MATLAB R2021b

Control components (Simulink) :
- Maximum torque per ampere ( optimal D-Q current calculation )
- D-Q current controllers ( integrated PD controllers for current control )
- Space vector modulation ( Switching vector based inverter control )
- Current based rotorflux estimation ( Calculates rotorflux amplitude and angle )
- U/f control ( Voltage-frequency control for speed control )
- Clarke, Park Inverse Park Transformations
- Infinite Impulse Response filter ( parameterizable based on cutting angular frequency )

HW components (Simscape) :
- Two level three phase voltage source inverter
- Induction machine

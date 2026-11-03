# Digital Control of DC-to-Three-Phase AC Conversion Based on GaN Power Devices

MATLAB/Simulink models, control code and results from the thesis. The models develop the MathWorks [Three-Phase Inverter Voltage Control example](https://www.mathworks.com/help/simscape-electrical/ug/three-phase-inverter-control.html) into voltage and current controllers, SPWM and SVPWM operation, and an RL-load system with sampled feedback. EPC2218 models are used for inverter simulation and device-loss calculations.

`Voltage control` and `Current control` contain the development models and their tests. `RL load` contains the 60 V models, with ADC tests and GaN calculations under `ADC`. `F28379D_Slow_Iteration` contains the embedded control files, LabVolt measurements and five hardware iterations. The laboratory power stage uses IGBTs; the GaN results are simulations and loss estimates.

Saved figures and tables use the numbering in the thesis.

# Digital Control of DC-to-Three-Phase AC Conversion Based on GaN Power Devices

The thesis develops voltage and dq current control for a three-phase inverter using SPWM and SVPWM in MATLAB/Simulink. The models include RL loads, ADC sampling and quantisation, and discrete PI control. EPC2218 GaN device models are used to calculate conduction and switching losses and estimate conversion efficiency.

Hardware tests use a TMS320F28379D controller and a LabVolt IGBT bridge. Recorded voltage and current measurements are used to update the PWM sequence between runs.

Models and saved results are organised into `Voltage control`, `Current control` and `RL load`, with ADC tests and GaN models under `RL load/ADC`. Hardware files are in `F28379D_Slow_Iteration`.

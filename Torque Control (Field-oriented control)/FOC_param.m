% CONSTANTS
Ts_sim = 1e-6;                              % Simulation stepsize for Simulink stepsize and Inverter circuit(Powergui)
SimTime = 4;                                % Simulation time
RPMtoRads = ( 2*pi ) / 60;                  % RPM to rad/s convert gain

% MOTOR PARAMETERS
Udc = 400;                                  % Inverter voltage source DC voltage
Lls = 0.0003495;                            % Stator leakage inductance [H]
Llr = 0.005473;                             % Rotor leakage inductance
Lm = 0.0354;                                % Magnetizing inductance %xxx
Rr = 0.6258;                                % Rotor resistance
Rs = 0.5968;                                % Stator resistance
PolePairs = 2;                              % Pole pairs
% ----------------for U/f speed control only-----------------------------------------------------------
UNomLL= 400;                                % motor nominal line - line voltage
NomFreq = 60;                               % motor nominal frequency (field weakening point frequency)
%----------------for U/f speed control only------------------------------------------------------------

% SPACE VECTOR MODULATION PARAMETERS
SVPWM_SampFreq = 1000;                      % Sample frequency to determinate sample time for calculating impulsewidth in any sector
SVPWM_SampTs= 1/SVPWM_SampFreq;
SVPWM_SwitchFreq = 20000;                   % Switching frequency to determinate switching time for inverter
SVPWM_SwitchTs = 1/SVPWM_SwitchFreq; 

% ROTORFLUX ESTIMATION PARAMETERS
RotFluxEst_Ts = Ts_sim;                     % Stepsize for rotor flux estimation
Lr = Llr + Lm;                              % Rotor inductance
Tr = Lr / Rr;                               % Rotor time constant

% D-Q CURRENT CONTROLLER PARAMETERS
CurrCntrl_Ki = 1e-4;                        % Integration factor  %0.0001
CurrCntrl_Kp = (2/3) * 6;                   % Proportional factor %0.00001;
CurrCntrl_Umax = Udc/sqrt(3);               % Maximum D-Q voltage limit
CurrCntrl_Umin = -Udc/sqrt(3);              % Minimum D-Q voltage limit
CurrCntrl_Ts = Ts_sim;

% MTPA CALCULATED CURRENT INPUTS + SPEED REFERENCE
%(Lowest current for the reference torque)
Id_ref =  21.7848;                           % D - axis current reference
Iq_ref =  19.8915;                           % Q - axis current reference

Speed_ref = 1000;                           % Mechanical speed reference
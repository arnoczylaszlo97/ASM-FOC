
Ts_sim = 1e-6;                              % Simulation stepsize for Simulink stepsize and Inverter circuit(Powergui)

% For three phase sinus generators
% Vrms = 230;                               % 1 fázis effektív értéke a max peak számításhoz
% Vpeak = Vrms*sqrt(2);                     % Input three phase AC voltages (Va_ref,Vb_ref,Vc_ref) peak voltage
% Amp = Vpeak;                              % Voltage amplitude ~ 326,6 V
% Freq = 50;                                % Input three phase AC voltages (Va_ref,Vb_ref,Vc_ref) frequency

% SPACE VECTOR MODULATION PARAMETERS
SVPWM_SampFreq = 1000;                      % Sample frequency to determinate sample time for calculating impulsewidth in any sector
SVPWM_SampTs= 1/SVPWM_SampFreq;
SVPWM_SwitchFreq = 20000;                   % Switching frequency to determinate switching time for inverter
SVPWM_SwitchTs = 1/SVPWM_SwitchFreq;                 

% ROTORFLUX ESTIMATION PARAMETERS
RotFluxEst_Ts = Ts_sim;                    % Stepsize for rotor flux estimation

% MOTOR PARAMETERS
Udc = 400;                                  % Inverter voltage source DC voltage
Llr = 0.005473;                             % Rotor leakage inductance
Lm = 0.0354;                                % Magnetizing inductance
Rr = 0.6258;                                % Rotor resistance
PolePairs = 2;                              % Pole pairs
% ----------------for U/f speed control only----------------------
UNomLL= 400;                                % motor nominal line - line voltage
NomFreq = 60;                               % motor nominal frequency (field weakening point frequency)
%----------------for U/f speed control only----------------------
Lr = Llr + Lm;                              % Rotor inductance
Tr = Lr / Rr;                               % Rotor time constant

% U/f CONTROL PARAMETERS
UFCtrl_UBelowLinReg = 5;                    % Voltage under linear region 
UFCtrl_LinRegPointFreq = 5;                 % Linear region point (f_min)
%UFCtrl_UFWPoint = UNomLL*sqrt(3);          % Field weakening point frequency
UFCtrl_FWPointFreq = NomFreq;               % motor nominal frequency = field weakening point frequency)
UFCtrl_MaxFreq = UFCtrl_FWPointFreq * 2;    % maximum frequency (in field weakening)
UFCtrl_Ts = Ts_sim;

% U/f CONTROL CONSTANTS
RPMtoRads = ( 2*pi ) / 60;                  % Gain to convert RPM to rads
NsToFs = PolePairs/60;                      % Gain to get electrical frequency from mechanical angular velocity[RPM]
FstoWm = 1/PolePairs * 2 * pi;              % Gain to get mechanical angular velocity[rad/s]  from electrical frequency

% FREQUENCY REFERENCE GENERATOR TEST PARAMETERS

Test_wTs = 1;                               % Stepsize for one speed reference
Test_NsCntr = 6;                            % Number of speed references
Test_NsArr = zeros();                       % Array of ws speed references

for i = 1 : 1 : Test_NsCntr
    fs = (UFCtrl_MaxFreq / Test_NsCntr) * i;
    Test_NsArr(i) = (60 * fs) / PolePairs;
end
%Test_NsArr(1) = -Test_NsArr(1);             % To test both direction
Test_NsMax = max(Test_NsArr);                % Highest ws speed reference
SimTime = Test_wTs * (Test_NsCntr);          % Simulation time
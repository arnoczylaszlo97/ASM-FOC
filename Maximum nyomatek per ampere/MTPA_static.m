close all;
clear all; 
clc;

% SIMSCAPE MOTOR PARAMETERS 
Rs = 0.5968;        % Stator resistance [ohm]
Rr = 0.6258;        % Rotor resistance [ohm]
Lls = 0.0003495;    % Stator leakage inductance [H]
Llr = 0.005473;     % Rotor leakage inductance [H]
p = 2;              % number of poles pairs
Lr = 0;             % Rotor inductance (calculated with Lm) [H]
Vdc = 400;          % DC voltage source [V]
f_rated = 60;       % rated frequency [Hz]
Lm = 0.0354;        % Magnetizing inductance [H] FOR NON-SATURATED LM!!!

% ------------------------------------------------------------------

% ------------------------------------------------------------------
% 2. MTPA PARAMETERS
% 21x21 matrix [wm,Te] - [ids,iqs]
% Nm : 0 - 20.000 RPM
% Te : 0 - 200 Nm 
% ------------------------------------------------------------------
wm = 3000;              % Reference rotor mechanical speed [RPM]
% ------------------------------------------------------------------
Te = 0;                 % Reference torque variable
Te_max = 20;            % Maximum reference torque 
Te_step = 20;           % Stepsize for torque
vi_condition = true;    % Current and voltage limit logical variable
i_step = 1e-1;          % Current vector amplitude step size [A] 
i_max = 80;             % Maximum current vector amplitude [A]
theta_step = 1e-2;      % Current vector angle [rad]
theta_max = pi/2;       % Maximum current vector angle [rad]
Lm_tol = 0.5;           % Magnetizing inductance relative tolerance [%]
Te_tol = 0.5;           % Developed torque relative tolerance [%]
i_prev = 0;             % Current vector amplitude for previous torque
cntr = 0;               % counter
% ------------------------------------------------------------------
% MTPA OUTPUTS
mtpa_ids = [];          % D - axis current MTPA points
mtpa_iqs = [];          % Q - axis current MTPA points
LmCalced_array = [];    % Calculated magnetizing inductance values
% ------------------------------------------------------------------
% MTPA MEASUREMENTS
ids_array  = [];        % D - axis current points for isotorque curve plots
iqs_array  = [];        % Q - axis current points for isotorque curve plots
Vamp_array = [];        % Calculated voltage amplitudes for [w,Te] reference
Iamp_array = [];        % Calculated current amplitudes for [w,Te] reference
Te_array   = [];
% ------------------------------------------------------------------

% MTPA SCRIPT
while( vi_condition && Te ~= Te_max ) 
    Te = Te + Te_step;
    cntr = cntr + 1;
    Imag_min = i_max;
    vi_condition = false;
    for  i = 0   : i_step  : i_max
        Imag = i;
        for j = 0: theta_step : theta_max   
            theta = j;
            ids = Imag * cos(theta);
            iqs = Imag * sin(theta);
            [im_mag ,idr, iqr,Psy_ds,Psy_qs] = ImMagnitude(iqs,ids,Lm,Lls,Llr,p,Te);              
            Ls = Lm + Lls;
            Lr = Lm + Llr;
            Te_calc = (3/2) * p * (Lm^2/Lr) * ids * iqs;
            if abs(Te - Te_calc) <= (Te/100)*Te_tol
                ids_array(end+1) = ids;
                iqs_array(end+1) = iqs;
                if Imag < Imag_min 
                    PsiM = im_mag * (Lm+Llr);
                    %Psy_dr = Lr * idr + Lm * ids;
                    [ws,w_slip] = wsCalc(wm, iqr, Rr, PsiM, p);
                    [Vamp,Vds,Vqs] = VoltageCalc(ids, iqs, idr, iqr, ws, Lm, Lls, Rs);
                    if Vamp <= Vdc/sqrt(3)
                        Imag_min = Imag;
                        vi_condition = true;
                        % ---MEASURMENTS FOR COMMAND WINDOW---
                        Te_array(cntr) = Te_calc
                        mtpa_ids(cntr) = ids
                        mtpa_iqs(cntr) = iqs
                        Vamp_array(cntr) = Vamp
                        Iamp_array(cntr)= Imag
                        LmCalced_array(cntr) = Lm
                        Vds
                        Vqs
                        im_mag
                        w_slip
                        wm * p *  2 * pi / 60
                        ws
                        idr
                        iqr
                        % ---MEASURMENTS FOR COMMAND WINDOW---
                        i_prev = Imag;            
                    end
                end               
            end
        end 
    end
end
% ------------------------------------------------------------------

% MTPA PLOT
h1 = figure;

plot(ids_array,iqs_array,'.','color','red')%,'MarkerSize',8,Color ="#FF0000");
hold on;
plot(mtpa_ids,mtpa_iqs,'o','color','blue');

P0 = [0 0];
for i = 1 : 1 :length(mtpa_ids)
     if( i ~= 1)
         P0 = [mtpa_ids(i-1),mtpa_iqs(i-1)];
     end
     Re = mtpa_ids(i);
     Im = mtpa_iqs(i);
     plot([P0(1), Re],[P0(2),Im], 'LineWidth',2,'color','black'); %LineWidth=2,Color="#03befc"
     hold on;
end
title('Maximum Torque per Ampere with static L_m =0.0354 [H], U_{max}= 230.94 [V], \omega_m = 1000 [RPM]');
grid on;
xlabel('I_{ds} [A]','FontSize',14);
ylabel('I_{qs} [A]','FontSize',14);
legend({'MTPA D-Q current points [A] ','Isotorque curves (\Delta = 10 [Nm]) ','MTPA points interpolated '},'Location','best','FontSize',10);
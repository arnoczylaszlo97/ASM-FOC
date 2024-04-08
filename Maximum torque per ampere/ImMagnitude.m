% This function shall calculate the magnetizing current magnitude.

% Given parameters:
% iqs    - Stator D - axis current [A]
% ids    - Stator Q - axis current [A]
% Lr     - Rotor leakage inductance [H]
% Te     - Tourque [Nm]
% p      - number of pole pairs

% Calculated parameters:
% Lr     - Rotor inductance [H]
% Psy_dr - Rotor D - axis flux [Wb]
% idr    - Rotor D - axis current [A]
% iqr    - Rotor Q - axis current [A]
% im     - Magnetizing current vector
% im_mag - Magnetizing current magnitude [A]

% Note : Rotor Q - axis flux is 0. ( Psy_qr = 0 )

% function [im_mag ,idr, iqr] = ImMagnitude(iqs,ids,Lm,Llr)
% Lr = Lm + Llr;
% % Calculating D - Q rotor currents
% iqr = -Lm * iqs/ Lr;
% idr = 0;
% % Magnetizing current vector magnitude
% im_comp = (ids + (1i * iqs)) + (idr + (1i * iqr));
% im_mag = abs(im_comp);
% 
% end

% IM CALC 2
function [im_mag ,idr, iqr,Psy_ds,Psy_qs] = ImMagnitude(iqs,ids,Lm,Lls,Llr,p,Te)
Lr = Lm + Llr;
Ls = Lm + Lls;
iqr = -Lm * iqs/ Lr;
Psy_qs = Ls * iqs + Lm * iqr;
Psy_ds = ((Te /(3/2 * p)) + (Psy_qs * ids))/iqs;
idr = (Psy_ds - (Ls *ids)) / Lm;
% Magnetizing current vector magnitude
% im_comp = (ids + (1i * iqs)) + (idr + (1i * iqr));
% im_mag = abs(im_comp);
im_mag = sqrt((ids + idr)^2+(iqs + iqr)^2);
end


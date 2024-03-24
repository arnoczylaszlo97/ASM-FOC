% This function shall calculate the magnetizing inductance for  on difference phase current - phase voltage pairs. 

% Iph_Array - Phase current array [A]
% Vph_Array - Phase voltage array [V]
% Rs - Stator resistance [Ohm]
% Lls - Stator leakage inductance [H]
% w_rated - nominal frequency of the motor

function [Lm_array] = NoLoadTest(Vph_array, Iph_array, Rs, Lls, w_rated)
    Lm_array = zeros();
    for i = 1 : 1 : length(Iph_array)
        Lm_array(i) = ( (sqrt( (Vph_array(i) / Iph_array(i))^2 - (Rs^2) )) / w_rated) - Lls;
    end
end
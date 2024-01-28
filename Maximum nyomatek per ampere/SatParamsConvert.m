% This function convert line RMS voltages,current to phase values.

% VL - Line voltage
% VP - Phase voltage
% VL = VL_rms × √2 
% VP = VL / √3
% VP = ( VL_rms × √2 ) / √3
% for currents, its the same

function [Vph_array,Iph_array] = SatParamsConvert(Vrms_array,Irms_array)
    Iph_array = zeros();
    Vph_array = zeros();
    for i = 1 : 1 : length(Irms_array)
        Iph_array(i) = Irms_array(i) * sqrt(2);
        Vph_array(i) = ( Vrms_array(i) * sqrt(2) ) /sqrt(3);
    end
end


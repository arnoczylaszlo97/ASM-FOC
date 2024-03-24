% This function shall plot the interpolated magnetizing inductance curve.
% x: Phase currents
% y: Magnetizing inductance

function [h] = PlotLmCurve(Iph_array, Lm_array, x_course, y_course)
    h = figure;
    grid on;
    title('Interpolated magnetizing inductance curve  ');
    xlabel('Magnetizing current [A]', 'FontSize', 14);
    ylabel('Magnetizing inductance [H]', 'FontSize' , 14);

    hold on;
    plot(Iph_array,Lm_array,'o');
    plot(x_course,y_course,'-r');
    legend({'I_m - L_m  points','I_m - L_m interpolated'},'Location','best','FontSize',10);
    hold off;
end
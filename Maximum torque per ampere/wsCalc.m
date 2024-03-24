function [ws,w_slip] = wsCalc(wm, iqr, Rr, PsiM, p )
    w_slip = abs(iqr * Rr / PsiM);
    RPMtoRad = 2 * pi / 60;
    ws = w_slip + ( wm * p *  RPMtoRad);
end
 


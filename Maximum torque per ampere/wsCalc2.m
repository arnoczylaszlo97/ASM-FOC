function [ws,w_slip] = wsCalc2(Lm, Psy_dr, Lr, Rr, iqs, wm, p )
    w_slip = Lm * iqs / (Lr + Rr) * Psy_dr;
    RPMtoRad = 2 * pi / 60;
    ws = w_slip + ( wm * p *  RPMtoRad);
end


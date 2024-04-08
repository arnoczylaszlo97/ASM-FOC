function [ Vamp,Vds,Vqs,Psy_ds,Psy_qs] = VoltageCalc(ids, iqs,idr,iqr, ws, Lm, Lls, Rs)

     % Stator d-q flux equations
     Ls = Lm + Lls;
     Psy_ds = Ls * ids + Lm * idr;
     Psy_qs = Ls * iqs + Lm * iqr;

     % Stator d-q voltage equation
     % STEADY STATE => Psy_ds * d/dt && Psy_qs * d/dt = 0
     Vds = Rs * ids - (ws * Psy_qs);
     Vqs = Rs * iqs + (ws * Psy_ds);

     % Vdc under limit check
     Vcomp = Vds + (1i * Vqs);
     Vamp = abs(Vcomp);
     %Vamp = sqrt(Vds^2 + Vqs^2);
end
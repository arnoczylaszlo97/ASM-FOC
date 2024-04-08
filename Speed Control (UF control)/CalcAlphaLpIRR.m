
% Calculating alpha coefficient for first-order IRR lowpass-filter with 
% the given cutoff frequency.

% Fc - cutoff frequency ( gain = 1/sqrt(2) ) [Hz]
% Ff - fundamental frequency ( gain ≈ 1 )    [Hz]
% Fs - sampling frequency ( gain ≈ 0 )       [Hz]

function [alpha] = CalcAlphaLpIRR(Ff,Fc,Fs)
   
    theta = 2 * pi *( Fc / Fs );
    c = cos(theta);
    alpha1 = ( -2 * c + 2  + sqrt( (2 * c - 2)^2 - 8*c + 8 ) ) / -2;
    alpha2 = ( -2 * c + 2  - sqrt( (2 * c - 2)^2 - 8*c + 8 ) ) / -2;

    if( abs(alpha1) > 0 && abs(alpha1) < 1 && abs(alpha1) < abs(alpha2))
        alpha = abs(alpha1);
    else
        alpha = abs(alpha2);
    end
    
    % Discrete transfer function
    numerator = alpha;
    denominator = [(-1 + alpha),1];
    Ts = 1/Fs;
    H = tf(numerator,denominator,Ts);
    
    % Bode plot
    w = [(Ff*2*pi) , (Fc*2*pi)];
    bode(H,w,'.-')
    hold on
    bode(H);
    grid on
   
end

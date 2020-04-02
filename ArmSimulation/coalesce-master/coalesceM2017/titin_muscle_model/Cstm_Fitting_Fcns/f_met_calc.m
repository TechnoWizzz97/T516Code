v = linspace(-vCE_max, vCE_max, 1000);

for i = 1:length(v)
    if v(i) >= 0
        f(i) = .23-.16*exp(-8*v(i));
    end
    if v(i) < 0
        f(i) = 0.01 - .11*v(i)+.06*exp(23*v(i));
    end
end  % Standard F

for i = 1:length(v)
    fl(i) = 0.01 - .11*v(i)+.06*exp(23*v(i));
end % Lower F
for i = 1:length(v)
    fu(i) = .23-.16*exp(-8*v(i));
end % Upper F

close all

plot(v,fl,v,fuv,f); legend('Lower Only','Upper Only','Standard');
plot(v,f,'k*'); hold on;
plot(v,fl, 'r'); plot(v,fu,'b'); legend('Std','Lower','Upper');
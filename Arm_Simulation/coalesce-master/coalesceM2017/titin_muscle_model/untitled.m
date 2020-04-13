close all;
g = 9.81;
h = 0.1;
v = 5;

t = 0:0.0001:1;

y = v.*t-1/2*g.*t.^2;

maxy = max(y);
i = find(y == maxy);
t(i)
t0 = v/g

plot(t,y)
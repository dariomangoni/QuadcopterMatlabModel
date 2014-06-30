a = 1;
b = 6;
c = -1;
d = 1;

delta0 = b^2 - 3*a*c;
delta1 = 2*b^3 -9*a*b*c +27*a^2*d;
C = ((delta1+sqrt(delta1^2-4*delta0^3))/2)^(1/3);

x = -(b+C+delta0/C)/3/a;
a*x^3+b*x^2+c*x+d


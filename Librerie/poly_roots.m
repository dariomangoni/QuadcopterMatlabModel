function [solutions] = poly_roots(coeff)

switch numel(coeff)
    case 5
        solutions = zeros(4,1);
        p4=coeff(5);
        p3=coeff(4);
        p2=coeff(3);
        p1=coeff(2);
        p0=coeff(1);

        p = p3/p4;
        q = p2/p4;
        r = p1/p4;
        s = p0/p4;

        a = 1;
        b = -q;
        c = p*r-4*s;
        d = 4*q*s-r^2-p^2*s;

        delta0 = b^2 - 3*a*c;
        delta1 = 2*b^3 -9*a*b*c +27*a^2*d;
        C = ((delta1+sqrt(delta1^2-4*delta0^3))/2)^(1/3);

        z1 = -(b+C+delta0/C)/3/a;

        R = sqrt(0.25*p^2-q+z1);
        if R == 0
            D = sqrt( 3/4*p^2-2*q+2*sqrt(z1^2-4*s) );
            E = sqrt( 3/4*p^2-2*q-2*sqrt(z1^2-4*s) );
        else
            D = sqrt(3/4*p^2-R^2-2*q+0.25*(4*p*q-8*r-p^3)/R );
            E = sqrt(3/4*p^2-R^2-2*q-0.25*(4*p*q-8*r-p^3)/R );
        end

        solutions(1) = -p/4 + 0.5*(R+D);
        solutions(2) = -p/4 + 0.5*(R-D);
        solutions(3) = -p/4 - 0.5*(R-E);
        solutions(4) = -p/4 - 0.5*(R+E);
        
        
    case 4
        
        solutions = zeros(3,1);

        a = coeff(4);
        b = coeff(3);
        c = coeff(2);
        d = coeff(1);

        delta0 = b^2 - 3*a*c;
        delta1 = 2*b^3 -9*a*b*c +27*a^2*d;
        C = ((delta1+sqrt(delta1^2-4*delta0^3))/2)^(1/3);

        solutions(1) = -(b+C+delta0/C)/3/a;
        u = (-1+1i*sqrt(3))/2;
        solutions(2) = -(b+C*u+delta0/C/u)/3/a;
        u = (-1-1i*sqrt(3))/2;
        solutions(3) = -(b+C*u+delta0/C/u)/3/a;
        
        
    case 3
        
        solutions = zeros(2,1);
        a=coeff(3);
        b=coeff(2);
        c=coeff(1);
        
        solutions(1) = (-b+sqrt(b^2-4*a*c))/2*a;
        solutions(2) = (-b-sqrt(b^2-4*a*c))/2*a;
        
    
    otherwise
        disp('error')
end
end
%======================================================================
%
%  COE-835  Controle adaptativo
%
%  Script para simular o exemplo
%
%  MRAC  : n  = 1     First order plant
%          n* = 1     Relative degree
%          np = 4     Adaptive parameters
%
%======================================================================
function dx=mrac(t,x)

global Ay By Aym Bym gamma w A km_1;

y      = x(1:2);
ym     = x(3:4);
theta  = x(5:end);

%--------------------------
r = 0;
for i=1:length(w)
    r = r + A(i)*sin(w(i)*t);
end


Omega2 = [km_1*r, km_1*r]';
u2 = theta(4:5)'*Omega2;

Omega1 = [km_1*r, km_1*r, u2]';
u1 = theta(1:3)'*Omega1;

u = [u1 u2]';
%------- Calculo de y --------
dy = Ay*y + By*u;

%------- Calculo de ym --------
dym = Aym*ym + Bym*[r r]';

e  = y - ym;
dtheta = -gamma*[Omega1'*e(1), Omega2'*e(2)]';

%--------------------------
dx = [dy' dym' dtheta']';    %Translation

%---------------------------
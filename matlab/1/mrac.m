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

global Ay By Cy Aym Bym gamma a1 a2 w1 w2 km;

xp      = x(1:2);
ym     = x(3:4);
theta  = x(5:end);

y = Cy*xp;
%--------------------------
r1 = 0;
r2 = 0;
for i=1:length(w1)
    r1 = r1 + a1(i)*sin(w1(i)*t);
    r2 = r2 + a2(i)*sin(w2(i)*t);
end


Omega2 = [km*r1, km*r2]';
u2 = theta(4:5)'*Omega2;

Omega1 = [km*r1, km*r2, u2]';
u1 = theta(1:3)'*Omega1;

u = [u1 u2]';
%------- Calculo de y --------
dxp = Ay*xp + By*u;

%------- Calculo de ym --------
dym = Aym*ym + Bym*[r1 r2]';

e  = y - ym;
dtheta = -gamma*[Omega1'*e(1), Omega2'*e(2)]';

%--------------------------
dx = [dxp' dym' dtheta']';    %Translation

%---------------------------
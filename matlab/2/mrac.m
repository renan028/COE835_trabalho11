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

global sysP sysM sysL gamma w A km_1;

xp     = x(1:4);
ym     = x(5:6);
uf     = x(7:8);
yf     = x(9:10);
theta  = x(11:end);
y = sysP.C*xp;
%--------------------------
r = 0;
for i=1:length(w)
    r = r + A(i)*sin(w(i)*t);
end

r_ = [r r]';

Omega2 = [uf' yf' y' r_']';
u2 = theta(10:end)'*Omega2;

Omega1 = [Omega2', u2]';
u1 = theta(1:9)'*Omega1;

u = [u1 u2]';

%% ------- Planta --------
dxp = sysP.A*xp + sysP.B*u;

%% ------- Modelo --------
dym = sysM.A*ym + sysM.B*[r r]';

%% Filtro
duf = sysL.A*uf + sysL.B*u;
dyf = sysL.A*yf + sysL.B*y;

%% erro
e  = y - ym;

%% Atualização
dtheta = -gamma*[Omega1'*e(1), Omega2'*e(2)]';

%% Transição
dx = [dxp' dym' duf' dyf' dtheta']'; 
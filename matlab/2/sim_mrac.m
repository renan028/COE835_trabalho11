%----------------------------------------------------------------------
%
%  COE-835  Controle adaptativo
%
%  Script para simular exemplo 
%
%  MRAC  : n  = 2, 3     Second and third order plant
%          n* = 1        Relative degree
%          np = 4, 6     Adaptive parameters
%
%                                                        Ramon R. Costa
%                                                        30/abr/13, Rio
%----------------------------------------------------------------------

global sysP sysM sysL gamma;

sim_str = 'sim0';
% options = odeset('OutputFcn','odeplot');
options = '';

%--------------------------------------------------- Simulation 1 (default)

sysP = ss(tf(Np_1,Dp_1));
sysM = ss(tf(Nm_1,Dm_1));
sysL = ss(tf(1,L));

% Initialization
y0  = [0 0 0 0]';
ym0 = [0 0]';
uf0 = [0 0]';
yf0 = [0 0]';
theta0 = zeros(17,1);
init = [y0' ym0' uf0' yf0' theta0']';

% Adaptation gain
gamma = gamma_1;

% Simulation
[T_1,X_1] = ode23s('mrac',tfinal,init,options);
xp_1      = X_1(:,1:4);
xm_1     = X_1(:,5:6);
theta_1 =  X_1(:,11:end);
modtt_1 = sqrt(sum(theta_1.^2,2));

y_1 = (sysP.C*xp_1')';
ym_1 = (sysM.C*xm_1')';
e0_1 =  y_1 - ym_1;


%----------------------------------------------------- Simulation 2 (gamma)

% changed = 1;
% 
% sysP = ss(tf(Np_1,Dp_1));
% sysM = ss(tf(Nm_1,Dm_1));
% 
% % Initialization
% y0  = [0 0 0 0]';
% ym0 = [0 0]';
% uf0 = [0 0]';
% yf0 = [0 0]';
% theta0 = zeros(17,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_2;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:4);
% ym_2     = X_2(:,5:6);
% theta_2 =  X_2(:,11:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (sysP.C*xp_2')';
% e0_2 =  y_2 - ym_2;
% 
% run plot_mrac.m;

%----------------------------------------------------- Simulation 3 (plant)

% changed = 2;
% 
% sysP = ss(tf(Np_2,Dp_2));
% sysM = ss(tf(Nm_1,Dm_1));
% 
% % Initialization
% y0  = [0 0 0 0]';
% ym0 = [0 0]';
% uf0 = [0 0]';
% yf0 = [0 0]';
% theta0 = zeros(17,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:4);
% ym_2     = X_2(:,5:6);
% theta_2 =  X_2(:,11:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (sysP.C*xp_2')';
% e0_2 =  y_2 - ym_2;
% 
% run plot_mrac.m;
% 
%----------------------------------------------------- Simulation 4 (model)

% changed = 3;
% 
% sysP = ss(tf(Np_1,Dp_1));
% sysM = ss(tf(Nm_2,Dm_2));
% 
% % Initialization
% y0  = [0 0 0 0]';
% ym0 = [0 0]';
% uf0 = [0 0]';
% yf0 = [0 0]';
% theta0 = zeros(17,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:4);
% ym_2     = X_2(:,5:6);
% theta_2 =  X_2(:,11:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (sysP.C*xp_2')';
% e0_2 =  y_2 - ym_2;
% 
% run plot_mrac.m;
% 
%-------------------------------------------------------- Simulation 5 (y0)

changed = 4;

sysP = ss(tf(Np_1,Dp_1));
sysM = ss(tf(Nm_1,Dm_1));

% Initialization
y0  = [1 0 1 0]';
ym0 = [0 0]';
uf0 = [0 0]';
yf0 = [0 0]';
theta0 = zeros(17,1);
init = [y0' ym0' uf0' yf0' theta0']';

% Adaptation gain
gamma = gamma_1;

% Simulation
[T_2,X_2] = ode23s('mrac',tfinal,init,options);
xp_2      = X_2(:,1:4);
ym_2     = X_2(:,5:6);
theta_2 =  X_2(:,11:end);
modtt_2 = sqrt(sum(theta_2.^2,2));

y_2 = (sysP.C*xp_2')';
e0_2 =  y_2 - ym_2;

run plot_mrac.m;
% 
% %---------------------------------------------------------------------

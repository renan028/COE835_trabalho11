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

global Ay By Cy Aym Bym Cym km gamma;

sim_str = 'sim0';
% options = odeset('OutputFcn','odeplot');
options = '';

%--------------------------------------------------- Simulation 1 (default)

N = Np_1*kp_1;
N = {N(1,1), N(1,2); N(2,1), N(2,2)};
D = {Dp_1, 1; 1, Dp_1};
sys = ss(tf(N,D));
Ay = sys.A;
By = sys.B;
Cy = sys.C;

km = km_1;
Nm = Nm_1*km_1;
Nm = {Nm(1,1), Nm(1,2); Nm(2,1), Nm(2,2)};
Dm = {Dm_1, 1; 1, Dm_1};
sys = ss(tf(Nm,Dm));
Aym = sys.A;
Bym = sys.B;

% Initialization
y0  = [0 0]';
ym0 = [0 0]';
theta0 = [0 0 0 0 0]';
init = [y0' ym0' theta0']';

% Adaptation gain
gamma = gamma_1;

% Simulation
[T_1,X_1] = ode23s('mrac',tfinal,init,options);
xp_1      = X_1(:,1:2);
ym_1     = X_1(:,3:4);
theta_1 =  X_1(:,5:end);
modtt_1 = sqrt(sum(theta_1.^2,2));

y_1 = (Cy*xp_1')';
e0_1 =  y_1 - ym_1;


%----------------------------------------------------- Simulation 2 (gamma)

% changed = 1;
% 
% N = Np_1*kp_1;
% N = {N(1,1), N(1,2); N(2,1), N(2,2)};
% D = {Dp_1, 1; 1, Dp_1};
% sys = ss(tf(N,D));
% Ay = sys.A;
% By = sys.B;
% Cy = sys.C;
% 
% Nm = Nm_1*km_1;
% Nm = {Nm(1,1), Nm(1,2); Nm(2,1), Nm(2,2)};
% Dm = {Dm_1, 1; 1, Dm_1};
% sys = ss(tf(Nm,Dm));
% Aym = sys.A;
% Bym = sys.B;
% 
% % Initialization
% y0  = [0 0]';
% ym0 = [0 0]';
% theta0 = [0 0 0 0 0]';
% init = [y0' ym0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_2;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:2);
% ym_2     = X_2(:,3:4);
% theta_2 =  X_2(:,5:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (Cy*xp_2')';
% e0_2 =  y_2 - ym_2;
% run plot_mrac.m;

%----------------------------------------------------- Simulation 3 (plant)

changed = 2;

N = Np_2*kp_2;
N = {N(1,1), N(1,2); N(2,1), N(2,2)};
D = {Dp_2, 1; 1, Dp_2};
sys = ss(tf(N,D));
Ay = sys.A;
By = sys.B;
Cy = sys.C;

km = km_1;
Nm = Nm_1*km;
Nm = {Nm(1,1), Nm(1,2); Nm(2,1), Nm(2,2)};
Dm = {Dm_1, 1; 1, Dm_1};
sys = ss(tf(Nm,Dm));
Aym = sys.A;
Bym = sys.B;

% Initialization
y0  = [0 0]';
ym0 = [0 0]';
theta0 = [0 0 0 0 0]';
init = [y0' ym0' theta0']';

% Adaptation gain
gamma = gamma_1;

% Simulation
[T_2,X_2] = ode23s('mrac',tfinal,init,options);
xp_2      = X_2(:,1:2);
ym_2     = X_2(:,3:4);
theta_2 =  X_2(:,5:end);
modtt_2 = sqrt(sum(theta_2.^2,2));

y_2 = (Cy*xp_2')';
e0_2 =  y_2 - ym_2;
run plot_mrac.m;

% 
%----------------------------------------------------- Simulation 4 (model)

% changed = 3;
% 
% N = Np_1*kp_1;
% N = {N(1,1), N(1,2); N(2,1), N(2,2)};
% D = {Dp_1, 1; 1, Dp_1};
% sys = ss(tf(N,D));
% Ay = sys.A;
% By = sys.B;
% Cy = sys.C;
% 
% km = km_2;
% Nm = Nm_2*km_2;
% Nm = {Nm(1,1), Nm(1,2); Nm(2,1), Nm(2,2)};
% Dm = {Dm_2, 1; 1, Dm_2};
% sys = ss(tf(Nm,Dm));
% Aym = sys.A;
% Bym = sys.B;
% Cym = sys.C;
% 
% % Initialization
% y0  = [0 0]';
% ym0 = [0 0]';
% theta0 = [0 0 0 0 0]';
% init = [y0' ym0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:2);
% ym_2     = X_2(:,3:4);
% theta_2 =  X_2(:,5:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (Cy*xp_2')';
% e0_2 =  y_2 - ym_2;
% run plot_mrac.m;
% 
% %-------------------------------------------------------- Simulation 5 (y0)
% 
% changed = 4;
% 
% N = Np_1*kp_1;
% N = {N(1,1), N(1,2); N(2,1), N(2,2)};
% D = {Dp_1, 1; 1, Dp_1};
% sys = ss(tf(N,D));
% Ay = sys.A;
% By = sys.B;
% Cy = sys.C;
% 
% km = km_1;
% Nm = Nm_1*km_1;
% Nm = {Nm(1,1), Nm(1,2); Nm(2,1), Nm(2,2)};
% Dm = {Dm_1, 1; 1, Dm_1};
% sys = ss(tf(Nm,Dm));
% Aym = sys.A;
% Bym = sys.B;
% Cym = sys.C;
% 
% % Initialization
% y0  = [1 0]';
% ym0 = [0 0]';
% theta0 = [0 0 0 0 0]';
% init = [y0' ym0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1;
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% xp_2      = X_2(:,1:2);
% ym_2     = X_2(:,3:4);
% theta_2 =  X_2(:,5:end);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% 
% y_2 = (Cy*xp_2')';
% e0_2 =  y_2 - ym_2;
% run plot_mrac.m;
% 
% %---------------------------------------------------------------------

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

global Ay By Aym Bym kp gamma w A gP gPm;

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
y_1      = X_1(:,1:2);
ym_1     = X_1(:,3:4);
theta_1 =  X_1(:,5:end);
modtt_1 = sqrt(sum(theta_1.^2,2));

r_1 = 0;
for i=1:length(A)
    r_1 = r_1 + A(i)*sin(w(i).*T_1);
end

e0_1 =  y_1 - ym_1;


%----------------------------------------------------- Simulation 2 (gamma)

changed = 1;

N = Np_1*kp_1;
N = {N(1,1), N(1,2); N(2,1), N(2,2)};
D = {Dp_1, 1; 1, Dp_1};
sys = ss(tf(N,D));
Ay = sys.A;
By = sys.B;

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
gamma = gamma_2;

% Simulation
[T_2,X_2] = ode23s('mrac',tfinal,init,options);
y_2      = X_2(:,1:2);
ym_2     = X_2(:,3:4);
theta_2 =  X_2(:,5:end);
modtt_2 = sqrt(sum(theta_2.^2,2));

r_1 = 0;
for i=1:length(A)
    r_1 = r_1 + A(i)*sin(w(i).*T_1);
end

e0_2 =  y_2 - ym_2;

run plot_mrac.m;

% %----------------------------------------------------- Simulation 3 (plant)
% 
% changed = 2;
% 
% P = P_2; % Plant parameters
% kp = kp_2;
% gP = gP_2;
% Pm = Pm_1; %Model parameters
% gPm = gPm_1;
% 
% [t1, tn, t2, t2n, L] = find2DOFparameters(P,Pm,A0); %Find ideal thetas and filter
% thetas = [t1, tn, t2, t2n];
% 
% ss_H = canon(ss(P), 'companion'); % Planta
% Ay = ss_H.A';
% By = ss_H.C';
% 
% ss_Hm = canon(ss(Pm), 'companion'); % Modelo
% Aym = ss_Hm.A';
% Bym = ss_Hm.C';
% 
% ss_uf = canon(ss(tf(1,L)), 'companion'); % Filtro u
% Auf = ss_uf.A';
% Buf = ss_uf.C';
% 
% ss_yf = canon(ss(tf(1,L)), 'companion'); % Filtro y
% Ayf = ss_yf.A';
% Byf = ss_yf.C';
% 
% % Initialization
% y0  = y0_1*ones(gP,1);
% ym0 = zeros(gPm,1);
% uf0 = zeros(gP-1,1);
% yf0 = zeros(gP-1,1);
% theta0 = zeros(2*gP,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1*eye(2*gP);
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% y_2      = X_2(:,1);
% ym_2     = X_2(:,gP+1);
% theta_2 =  X_2(:,3*gP+gPm-1:end);
% tiltheta_2 = theta_2 - thetas.*ones(length(theta_2),2*gP);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% e0_2 =  y_2 - ym_2;
% r_2 = 0;
% for i=1:length(A)
%     r_2 = r_2 + A(i)*sin(w(i).*T_2);
% end
% 
% run plot_mrac.m;
% 
% %----------------------------------------------------- Simulation 4 (model)
% 
% changed = 3;
% 
% P = P_1; % Plant parameters
% kp = kp_1;
% gP = gP_1;
% Pm = Pm_2; %Model parameters
% gPm = gPm_2;
% 
% [t1, tn, t2, t2n, L] = find2DOFparameters(P,Pm,A0); %Find ideal thetas and filter
% thetas = [t1, tn, t2, t2n];
% 
% ss_H = canon(ss(P), 'companion'); % Planta
% Ay = ss_H.A';
% By = ss_H.C';
% 
% ss_Hm = canon(ss(Pm), 'companion'); % Modelo
% Aym = ss_Hm.A';
% Bym = ss_Hm.C';
% 
% ss_uf = canon(ss(tf(1,L)), 'companion'); % Filtro u
% Auf = ss_uf.A';
% Buf = ss_uf.C';
% 
% ss_yf = canon(ss(tf(1,L)), 'companion'); % Filtro y
% Ayf = ss_yf.A';
% Byf = ss_yf.C';
% 
% % Initialization
% y0  = y0_1*ones(gP,1);
% ym0 = zeros(gPm,1);
% uf0 = zeros(gP-1,1);
% yf0 = zeros(gP-1,1);
% theta0 = zeros(2*gP,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1*eye(2*gP);
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% y_2      = X_2(:,1);
% ym_2     = X_2(:,gP+1);
% theta_2 =  X_2(:,3*gP+gPm-1:end);
% tiltheta_2 = theta_2 - thetas.*ones(length(theta_2),2*gP);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% e0_2 =  y_2 - ym_2;
% r_2 = 0;
% for i=1:length(A)
%     r_2 = r_2 + A(i)*sin(w(i).*T_2);
% end
% 
% run plot_mrac.m;
% 
% %-------------------------------------------------------- Simulation 5 (y0)
% 
% changed = 4;
% 
% P = P_1; % Plant parameters
% kp = kp_1;
% gP = gP_1;
% Pm = Pm_1; %Model parameters
% gPm = gPm_1;
% 
% [t1, tn, t2, t2n, L] = find2DOFparameters(P,Pm,A0); %Find ideal thetas and filter
% thetas = [t1, tn, t2, t2n];
% 
% ss_H = canon(ss(P), 'companion'); % Planta
% Ay = ss_H.A';
% By = ss_H.C';
% 
% ss_Hm = canon(ss(Pm), 'companion'); % Modelo
% Aym = ss_Hm.A';
% Bym = ss_Hm.C';
% 
% ss_uf = canon(ss(tf(1,L)), 'companion'); % Filtro u
% Auf = ss_uf.A';
% Buf = ss_uf.C';
% 
% ss_yf = canon(ss(tf(1,L)), 'companion'); % Filtro y
% Ayf = ss_yf.A';
% Byf = ss_yf.C';
% 
% % Initialization
% y0  = y0_2*ones(gP,1);
% ym0 = zeros(gPm,1);
% uf0 = zeros(gP-1,1);
% yf0 = zeros(gP-1,1);
% theta0 = zeros(2*gP,1);
% init = [y0' ym0' uf0' yf0' theta0']';
% 
% % Adaptation gain
% gamma = gamma_1*eye(2*gP);
% 
% % Simulation
% [T_2,X_2] = ode23s('mrac',tfinal,init,options);
% y_2      = X_2(:,1);
% ym_2     = X_2(:,gP+1);
% theta_2 =  X_2(:,3*gP+gPm-1:end);
% tiltheta_2 = theta_2 - thetas.*ones(length(theta_2),2*gP);
% modtt_2 = sqrt(sum(theta_2.^2,2));
% e0_2 =  y_2 - ym_2;
% r_2 = 0;
% for i=1:length(A)
%     r_2 = r_2 + A(i)*sin(w(i).*T_2);
% end
% 
% run plot_mrac.m;
% 
% %---------------------------------------------------------------------

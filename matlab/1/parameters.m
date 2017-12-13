clear;
clc;
close all;
global a1 a2 w1 w2;

PRINT = true;
% PRINT = false;

%% 1nd order system

%Simulation time
tfinal = 15;

%Reference
w1 = [0.63493 4.5669];
w2 = [0.1 1.1];
a1 = [1 1];
a2 = [1 1];

%--------------- First set of parameters ----------------

%P
phi_1 = pi/3;
h_1 = 1;
Np_1 = [1 0;0 1];
Dp_1 = [1 2];
kp_1 = [cos(phi_1) sin(phi_1);-h_1*sin(phi_1) h_1*cos(phi_1)];

%Pm
Nm_1 = [1 0;0 1];
Dm_1 = [1 2];
km_1 = 2;
gPm_1 = length(Dm_1)-1;

%Initial conditions
y0_1  = 0;

%Adaptation gain
gamma_1 = 10;

%--------------- Second set of parameters ----------------

%P
phi_2 = pi;
h_2 = 2;
Np_2 = [1 0;0 1];
Dp_2 = [1 2];
kp_2 = [cos(phi_2) sin(phi_2);-h_1*sin(phi_2) h_1*cos(phi_2)];

%Pm
Nm_2 = [2 0;0 2];
Dm_2 = [1 2];
km_2 = 1;

%Initial conditions
y0_2  = 10;

%Adaptation gain
gamma_2 = 1;

run sim_mrac.m;

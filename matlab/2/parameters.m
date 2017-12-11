clear;
clc;
close all;
global A w km_1 L;

PRINT = true;
% PRINT = false;

%% 1nd order system

%Simulation time
tfinal = 100;

%Reference
w = [0.63493 4.5669];
A = [1 1];

%--------------- First set of parameters ----------------

%P
Np_1 = {[1 3],[2 0];[-2 -4], [1 3]};
Dp_1 = {[1 0 -1],[1 0 -1];[1 0 -1],[1 0 -1]};

%Pm
km_1 = [2 1];
Nm_1 = {[km_1(1)], [0];[0], [km_1(2)]};
Dm_1 = {[1 2], [1];[1], [1 1]};

% Filtro 
L = [1 1];

%Initial conditions
y0_1  = 0;

%Adaptation gain
gamma_1 = 20;

%--------------- Second set of parameters ----------------

%P
Np_1 = {[1 2],[1 0];[-1 -2], [1 2]};
Dp_1 = {[1 -2 1],[1 -2 1];[1 -2 1],[1 -2 1]};

%Pm
km_1 = [1 1];
Nm_1 = {[km_1(1)], [0];[0], [km_1(2)]};
Dm_1 = {[1 1], [1];[1], [1 1]};

%Initial conditions
y0_2  = 10;

%Adaptation gain
gamma_2 = 1;

run sim_mrac.m;

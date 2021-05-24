%---------------------------------------------------------------------------%
%                            PROGRAM DESCRIPTION                            |
%---------------------------------------------------------------------------%
% This script allows the implementation of Model Predictive Controller 
% (MPC) in a modular approach. Program settings are described in each block.
%
% To succesfully run this program, the following project structure must be
% mantained:
%
% Required File Structure:
% Proyect/Main.m  : (This File)
% Proyect/SoMs    : This folder contains the simulation oriented models.
% Proyect/CoMs    : This folder contains the control oriented models.
% Proyect/Dist    : This folder contains the disturbance models, if any.
% Proyect/Results : This folder contains the results obtained 
%
%
% Reserved file/variable names:


%---------------------------------------------------------------------------%
%                                   SETUP                                   |
%---------------------------------------------------------------------------%
% Initialize; Clear screen, delete workspace and close figures. 
close all; clc; clear;


% Current Folder and Set new User Path;
filePath    = matlab.desktop.editor.getActiveFilename;
fileName    = filePath(find(filePath=='/',1,'last')+1:end);
proyectPath = filePath(1:length(filePath)-length(char(fileName))); 
addpath([proyectPath,'/CoMs']); 
userpath(proyectPath);
cd(proyectPath)

% Import Casadi and Yalmip
import casadi.*

%---------------------------------------------------------------------------%
%                              PROGRAM SETTINGS                             %
%---------------------------------------------------------------------------%
 
saveResults = 0; % if true, plots will be saved inside the results folder.
 
% Define control variables to run described program blocks accordingly.

% -------------------- Model Inputs and States Definition ------------------%

% BATTERY
bat_nom_V = 40;     % Battery nominal voltage [V]
capacity  = 2.2;    % Battery rated capacity  [Ah]
% FUEL CELL

% DISTURBANCE SIGNAL
% Select: 1-4
powerProfile = 1; 


% ------------------------ Simulation Oriented Model -----------------------%
% The SoM file is assumed to be a simulink file (.slx) containing the model.
% Inputs to the model should be specified as x1,x2,...,xn

% Define name and parameters of the SoM:
SoM_name = 'SA_50_FAST';

% -------------------------- Control Oriented Model ------------------------%
% The CoM file is assumed to be a matlab script (.m) where system parameters
% are defined, additionally the discretized state space matrices must also
% be definied and initiated.

% Define name: 

% Select:
%  'SA_50_nonLinear'
%  'SA_50_linear'

CoM_name = 'SA_50_linear2'; %[!]

% ------------------------------ MPC Parameters ----------------------------%
% Define controller parameters.
% Hp = 
% ts = 
% System Constraints: Define state/input constraints adhereing to the
% following structure.
%        constraint = [state/input number, minValue, maxValue ; .... ]
%        e.g.     C = [1, -inf, 0;  3, sqrt(a^2+b^2), 50]

% Constraint on input and states;
SoC_Max    = 80;   % Percent
SoC_Min    = 20;   % Percent
C_rate_Max = inf;  % h^-1
C_rate_Min = -inf; % h^-1
uMax       = 50;   % Watts
uMin       = 20;    % Watts


% -------------------------- Simulation Parameters -------------------------%
% Define controller parameters

Ts      = 1;
Hp      = 25;

%---------------------------------------------------------------------------%
%                               PROGRAM BLOCKS                              %
%---------------------------------------------------------------------------%


% -------------------------- Simulation Parameters -------------------------%
fileName = ['Ts_',num2str(Ts),'__','Hp_',num2str(Hp),'.png'];
sim('SA_50')
        
% ------------------------------ Plot Settings -----------------------------%

f = figure('visible','on');
% Plot 
subplot(6,3,[1:6])
plot(P_fc_hist,'k','LineWidth',1.8);hold on; grid; 
plot(P_load_hist,'r','LineWidth',1.8);
plot(P_fc_ref_hist,'b','LineWidth',1.8);

xlabel('Time [s]');ylabel('Power [W]'); 
lg=legend('Fuel Cell Power Output','Load Power Output','Reference FC Power Output');
lg.FontSize=8;
title('Power in the System')

subplot(6,3,[10:12])
plot(SOC_hist,'k','LineWidth',1.8); grid; 
xlabel('Time [s]');ylabel('Charge [%]'); 
title('Battery State of Charge');

subplot(6,3,[16:18])
plot(C_rate_hist.Time(50:end),C_rate_hist.Data(50:end),'k','LineWidth',1.8); grid; 
xlabel('Time [s]');ylabel('C_rate [h^1]'); 
title('Battery C-rate');

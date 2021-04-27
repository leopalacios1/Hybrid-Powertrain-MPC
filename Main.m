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
userpath(proyectPath);
cd(proyectPath)

%---------------------------------------------------------------------------%
%                              PROGRAM SETTINGS                             %
%---------------------------------------------------------------------------%
 
saveResults = 0; % if true, plots will be saved inside the results folder.
 
% Define control variables to run described program blocks accordingly.

% -------------------- Model Inputs and States Definition ------------------%


% ------------------------ Simulation Oriented Model -----------------------%
% The SoM file is assumed to be a simulink file (.slx) containing the model.
% Inputs to the model should be specified as x1,x2,...,xn

% Define name and parameters of the SoM:
SoM_name = 'SA_50'; %[!]

% -------------------------- Control Oriented Model ------------------------%
% The CoM file is assumed to be a matlab script (.m) where system parameters
% are defined, additionally the discretized state space matrices must also
% be definied and initiated.

% Define name and parameters of the CoM 
CoM_name = 'SA_50'; %[!]

% --------------------------- Load Disturbance Signal ----------------------%
% The CoM file is assumed to be a matlab script (.m) where system parameters
% are defined, additionally the discretized state space matrices must also
% be definied and initiated.

Dist_profile = 'powerProfile1'; %[!]
disturbance  = load(Dist_profile);

% ------------------------------ MPC Parameters ----------------------------%
% Define controller parameters.
% Hp = 
% ts = 
% System Constraints: Define state/input constraints adhereing to the
% following structure.
%        constraint = [state/input number, minValue, maxValue ; .... ]
%        e.g.     C = [1, -inf, 0;  3, sqrt(a^2+b^2), 50]
stateConstraints = 
inputConstraints = 

% -------------------------- Simulation Parameters -------------------------%
% Define controller parameters

% ------------------------------ Plot Settings -----------------------------%
% Define controller parameters

%---------------------------------------------------------------------------%
%                               PROGRAM BLOCKS                              %
%---------------------------------------------------------------------------%

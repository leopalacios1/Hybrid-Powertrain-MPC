%---------------------------------------------------------------------------%
%                            PROGRAM DESCRIPTION                            |
%---------------------------------------------------------------------------%
% This script allows the implementation of Model Predictive Controller 
% (MPC) in a modular approach. Program settings are described in each block.
%
% To succesfully run this program, the following structure must be
% mantained:
%
% Required Files:
%
%
% Reserved file/variable names:
% CoM
% SoM

%---------------------------------------------------------------------------%
%                                   SETUP                                   |
%---------------------------------------------------------------------------%
% Initialize 
close all; clc; clear;
% Current Folder and Set new User Path;
fileName    = 'Main.m'; 
filePath    = matlab.desktop.editor.getActiveFilename;
proyectPath = filePath(1:length(filePath)-length(char(fileName)));
userpath(proyectPath)
cd(proyectPath)

%---------------------------------------------------------------------------%
%                              PROGRAM SETTINGS                             %
%---------------------------------------------------------------------------%
% Define control variables to run described program blocks accordingly.


% ------------------------ Simulation Oriented Model -----------------------%
% Define name and parameters of the SoM:
SoM_name = 'SA_50';

% -------------------------- Control Oriented Model ------------------------%
% Define name and parameters of the CoM 
SoM_name = 'SA_50';

% ------------------------------ MPC Parameters ----------------------------%
% Define controller parameters.

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

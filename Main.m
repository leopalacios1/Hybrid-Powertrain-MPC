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
% Proyect/MPCs    : This folder contains the MPCs, each with its own CoM.
% Proyect/Dist    : This folder contains the disturbance models, if any.
% Proyect/Results : This folder contains the results obtained 
% Proyect/Dist    : This folder contains files defining system parameters.
%

%---------------------------------------------------------------------------%
%                                   SETUP                                   |
%---------------------------------------------------------------------------%
% Initialize; Clear screen, delete workspace and close figures. 
clc; clear; close all;

% Necessary path management;
filePath    = matlab.desktop.editor.getActiveFilename;
fileName    = filePath(find(filePath=='/',1,'last')+1:end);
proyectPath = filePath(1:length(filePath)-length(char(fileName))); 
addpath([proyectPath,'/MPCs']); 
addpath([proyectPath,'/PARAMETERS']);
addpath([proyectPath,'/SoMs']);
addpath([proyectPath,'/CoMs']);
userpath(proyectPath);
cd(proyectPath);

% Define global varibale to be used by files throughout the project.
global Ts SoC_Max SoC_Min C_rate_Max C_rate_Min uMax uMin;
global Hp Q R;


% Import Casadi and Yalmip
import casadi.*


%---------------------------------------------------------------------------%
%                              PROGRAM SETTINGS                             %
%---------------------------------------------------------------------------%
% In this section, controller, simulation and distrubance parameters are defined.
% To study controller performance, change parameters accordingly.

% ------------------------ Overall System Parameters ---------------------- %

% DISTURBANCE SIGNAL
% Select: 1-4
powerProfile = 4; 

% BATTERY INITIAL CONDITIONS
SoC_init   = 50    ;   % Initial state of charge.        [%]

% ------------------------ Simulation Oriented Model -----------------------%
% The SoM_name file is assumed to be a simulink file (.slx) containing the model.
% SoM_param .m file is where model parameters are defined.

% SoM Name, select 'SA_50_SIMPLE_MOSFET' / 'SA_50_FULL_MOSFET'
SoM_name  = 'SA_50_SIMPLE_MOSFET';
SoM_param = 'parameterConfig1';

% -------------------------- Control Oriented Model ------------------------%
% The CoM file is assumed to be a matlab script (.m) where system parameters
% are defined, additionally the discretized state space matrices must also
% be definied and initiated.

% Define name [ 1: linear /  2: nonlinear ]: 
modelSelection = 1;


% ------------------------------ MPC Parameters ----------------------------%
% Define controller parameters here.

% State References

socRef     = 50;        % State of charge reference             [%]
crateRef   = 0;         % C-rate reference                      [h^-1]

% Constraint on input and states;
SoC_Max    = 80;        % Maximum State of Charge               [%]
SoC_Min    = 20;        % Minimum State of Charge               [%]
C_rate_Max = inf;       % Maximim C-rate                        [h^-1]
C_rate_Min = -inf;      % Minumum C-rate                        [h^-1]

% Linear Controller Constraints
uMax       = 60;        % Maximum allowed FC power              [W]
uMin       = 20;        % Minimum allowed FC power              [W]

% Nonlinear Controller Constraints
uMax_nl    =  000 ;     % Maximum allowed FC current            [A]
uMin_nl    =  000 ;     % Minumum allowed FC current            [A]

% Sampling time and Prediction Horizon Definition
TsVec      = [1]  ;     % Controller Sampling Times             [s]
HpVec      = [40] ;     % Prediction Horizon Sampling Times     [#]

% -------------------------- Simulation Parameters -------------------------%
simTime    = 2000 ;


%---------------------------------------------------------------------------%
%                               PROGRAM (DO NOT CHANGE)                     %
%---------------------------------------------------------------------------%

% Load System Parameters from parameter configuration file.
eval(SoM_param)

% Load Linear Model from configuration file.
if modelSelection == 1
    modelFile  = 'linearModel';
    configFile = 'linearMPCconfig';
elseif modelSelection == 2
    modelFile  = 'nonLinearModel',
end

% Figure index
fIdx = 1;

% Results folder name
dateTime = clock;
cd RESULTS/ 
folderName = [num2str(dateTime(1)),'-',num2str(dateTime(2)),'-',num2str(dateTime(3)) ...
    ,'_',num2str(dateTime(4)),'h',num2str(dateTime(5)),'m'];
mkdir(folderName);
cd(folderName);

for Ts = TsVec
    for Hp = HpVec  %[20,30]
      for q_soc = 20  %[80,500]
          for q_crate = 600  %[1,80,500]
              for r = 50    %[80,500]
                    
                    % MPC's Weights
                    Q=diag([q_soc q_crate]);   
                    R=diag([r]); 
                  
                    % Define optimizer controller for Yalmip
                    if modelSelection == 1
                       eval(modelFile)
                       eval(configFile)
                    end
                    
                    % Filename for results output.
                    fileName   = ['HP',num2str(Hp),'_Qa',num2str(q_soc),'_Qb',num2str(q_crate),'_R',num2str(r)];
                    
                    % Simulate system:
                    t_init  = now;
                    SimResults  =  sim(SoM_name);
                    f_final = now;
                    simCompTime = (f_final - t_init) * 3600 *24;
                    
                    % Get results:
                    C_rate_hist     = SimResults.C_rate_hist;   
                    P_fc_hist       = SimResults.P_fc_hist;     
                    P_fc_ref_hist   = SimResults.P_fc_ref_hist; 
                    P_load_hist     = SimResults.P_load_hist;   
                    SOC_hist        = SimResults.SOC_hist;      
                    time            = SimResults.time;          
                    tout            = SimResults.tout; 
                    CompTime_hist   = SimResults.CompTime_hist; 
                    I_bat_hist      = SimResults.I_bat; 
                    V_bat_hist      = SimResults.V_bat;
                    
                    maxCompTime     = max(CompTime_hist);
                    meanCompTime    = mean(CompTime_hist);
                    stdCompTime     = std(CompTime_hist);
                    
                    disp(['For a prediction horizon of ',num2str(Hp),' and sampling time of ',num2str(Ts),'s: '])
                    disp(['Maximum computational time: ',num2str(maxCompTime)])
                    disp(['Mean computational time: ',num2str(meanCompTime)])
                    disp(['Standar deviation: ',num2str(stdCompTime)])
                    disp(' ')
                    disp(['For a simulation time of ',num2str(simTime), 's, it took ',num2str(simCompTime), ' to simulate.'])
                    
                    % Plot Results
                    f(fIdx) = figure('visible','on');

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

                    saveas(f(fIdx),[fileName,'.png'])
                    saveas(f(fIdx),[fileName,'.fig'])
                    disp(['Done with: Ts_',fileName])
                    fIdx=fIdx+1;

              end

          end

      end

    end
end

cd ..
cd ..


% Initialize; Clear screen, delete workspace and close figures. 
close all; clc; clear;
format short

%%

% Current Folder and Set new User Path;
filePath    = matlab.desktop.editor.getActiveFilename;
fileName    = filePath(find(filePath=='/',1,'last')+1:end);
proyectPath = filePath(1:length(filePath)-length(char(fileName))); 
userpath(proyectPath);
cd(proyectPath)

% PARAMETERS
Q    = 2.2;             % Battery rated capacity (Ah)
Vbat = 43.075;          % Battery Voltage, assumed constant (V)
b    = 1/(3600*Q*Vbat); % Constant
initialStates = [50; 0];  % [SoC C-rate];

% Times
timeSpan = 200;   % simulation runtime
mpcTs    = 0.5;   % MPC sampling time
simTs    = 5e-5;  % Discrete simulation sampling time

% LOAD POWER PROFILE
cd ..
cd Dist
powerProfile = load('powerProfile1.mat');
cd ..
cd SoMs

% ADJUST DISTURBANCE TO SAMPLING TIME
ts = mpcTs;                % MULTIPLE OF 0.1
powerProfile2              = powerProfile;
powerProfile2.xData        = powerProfile.xData(1:ts/powerProfile.SamplingTime:end);
powerProfile2.yData        = powerProfile.yData(1:ts/powerProfile.SamplingTime:end);
powerProfile2.SamplingTime = ts;

%% RESPONSE - SoM

% Initial Conditions
SoC_0          = initialStates(1);
% Nonlinear Simulation
startSim  = tic; % Start simulation
Time      = sim('SA_50');
simTime   = toc(startSim);
disp(['Simulation took ',num2str(simTime),' seconds.'])

SOC_SoM    = SoC;
iBAT_SoM   = iBat;
C_rate_SoM = abs(iBAT_SoM)./Q;


% ADJUST VALUES TO SAMPLING TIME FOR COMPARISON
SOC_SoM_adj     = SOC_SoM(1:mpcTs/simTs:end);
iBAT_SoM_adj    = iBAT_SoM(1:mpcTs/simTs:end);
C_rate_SoM_adj  = C_rate_SoM(1:mpcTs/simTs:end);
Time_adj        = Time(1:mpcTs/simTs:end);

figure;
% Plot SoC Response
subplot(1,2,1)
plot(Time_adj,SOC_SoM_adj,'k','LineWidth',1.8);hold on; 
xlabel('time [s]');ylabel('SOC: Simulation [%]'); 
%legend('h1 plant','h1 controller','h1 ref','h1 max','h1 min');
title('State of Charge Response')

% Plot C-rate Response
subplot(1,2,2)
plot(Time_adj,C_rate_SoM_adj,'k','LineWidth',1.8);hold on; 
xlabel('time [s]');ylabel('C-rate: Simulation [h^-1]'); 
%legend('h1 plant','h1 controller','h1 ref','h1 max','h1 min');
title('C-rate Response')

%% RESPONSE - CoM

% CoM
A       = [1 100; 0 0];
B       = [0;b];
Bd      = [0;-b];
C       = eye(2);
D       = 0;

nSteps   = round(timeSpan/mpcTs);

for i = 1:nSteps
    
    % Simulate Linear Model __________________________ 
    X_l = A*(CurrentState)+B*(U);
   
    % Calculate State: C-rate
    
    % Simulate Linear Model __________________________ 
    X_l = A*(CurrentState)+B*(U);
    
    % Store information ______________________________
    % Add States
    finalState    = [h1_h2_nonlinear;h3_h4_nonlinear];
    statesHist_nl = [statesHist_nl, finalState];
    statesHist_l  = [statesHist_l, X_l];
    
    % Add Control Action
    controlHist = [controlHist,U];
    
    % Add Controller Optimization Time
    CalcTime = [CalcTime calcTime];
    disp(['Timestep: ',num2str(i)]);
end

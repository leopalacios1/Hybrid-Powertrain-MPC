% Initialize 
close all; clc; clear;
% Current Folder and Set new User Path;
fileName    = 'CRA1_Carrizosa-Palacios.m'; 
filePath    = matlab.desktop.editor.getActiveFilename
proyectPath = filePath(1:length(filePath)-length(char(fileName)))
userpath(proyectPath);

% SYSTEM MODEL - SYMBOLIC ______________________________________________________________________________

% MPC Parameters _______________________________________________________________________________________

% SYSTEM PARAMETERS ____________________________________________________________________________________

% REALIZATION OF SYSTEM AND EQUILIBRIUM POINTS _________________________________________________________
%%
% LOAD DISTURBANCE (PLAY LIKE GOD)
clc;

ts   = 1;               % Sampling time (s)
Q    = 2.2;             % Battery rated capacity (Ah)
Vbat = 43.075;          % Battery Voltage, assumed constant (V)
b    = 1/(3600*Q*Vbat)  % Constant

A = [1 100; 0 0];
B = [0;b];
C = eye(2);
D = 0;

SYS_CoM_ct = ss(A,B,C,D)
% Discretize model
SYS_CoM_dt = c2d(SYS_CoM_ct,ts,'tustin')                 % LATER PASSED TO MPC 

  
%%
% SYSTEM CONSTRAINTS ___________________________________________________________________________________

nStates = size(A_lin,1);
nInputs = size(B_lin,2);

% State Constraints - CHANGE AS NEEEDED
stateMax_global = 15; % Cm     
stateMin_global = 0;  % Cm 

% 
StateConstMax_global = repmat(stateMax_global,nStates,1); % LATER PASSED TO MPC 
StateConstMin_global = repmat(stateMin_global,nStates,1); % LATER PASSED TO MPC 

% Input Constraints - CHANGE AS NEEEDED
inputMax_global = 10; % Volts
inputMin_global = 0;  % Volts

InputConstMax_global = repmat(inputMax_global,nInputs,1); % LATER PASSED TO MPC 
InputConstMin_global = repmat(inputMin_global,nInputs,1); % LATER PASSED TO MPC 

% CONTROLLER SETTING __________________________________________________________________________________

statePenalty = [1 40];                                       % LATER PASSED TO MPC                 
inputPenalty = [1 100];                                      % LATER PASSED TO MPC             
CurrentState = zeros(nStates,1);                             % LATER PASSED TO MPC  (THIS WILL BE INSIDE SIMULATION LOOP)           
Hp = 25  ;  % prediction horizon

% REFERENCE SETTING
h1_ref_value = [h1_0+1, 3];
h1_ref_time  = [0 ,120];

h2_ref_value = [h2_0+3, 1];
h2_ref_time  = [0 ,220];


% SIMULATION   _________________________________________________________________________________________

ts = ts;       % sampling time - ASSIGN BEFORE MODEL LINEARIZATION (line 67)
timeSpan = 400; % simulation runtime
nSteps   = round(timeSpan/ts);

% Nonlinear model response (recall input is relative to equilibrium)
statesHist_nl = zeros(nStates,1);     % each colum represents state at time step

% Linear model response (recall input is relative to equilibrium)
statesHist_l = zeros(nStates,1);      % each colum represents state at time step

% Controller signal and computational time history
controlHist  = [0;0];             % each colum represents input at time step
CalcTime = [0];

% LOGIC FOR CHANGING REFERENCES ___________________
REF1 = zeros(1,timeSpan+1);
REF2 = zeros(1,timeSpan+1);
for i = 1:nSteps
    h1_ref = 0;
    h2_ref = 0;
    for j = 1:length(h1_ref_time)
        if h1_ref_time(j) <= i*ts
            h1_ref = h1_ref + h1_ref_value(j);
        end
    end
    for j = 1:length(h2_ref_time)
        if h2_ref_time(j) <= i*ts
            h2_ref = h2_ref + h2_ref_value(j);
        end
    end
    REF1(i) = h1_ref;
    REF2(i) = h2_ref; 
end

% Start simulation
startSim = tic;
% Define variables that avoid issues when commenting out controller for step response
diagnostics = 0;
calcTime = [];
for i = 1:nSteps
    
    Reference = [REF1(i);REF2(i)];
    % Get Measured State (State of nonlinear plant)____
    CurrentState = statesHist_nl(:,i);
    
    %Determine next control action________MPC_________ 
    [U,calcTime, diagnostics] = MPC_fun2(SYS_CoM,EquilibriumPoints,EquilibriumInputs, StateConstMax_global,...
    StateConstMin_global,InputConstMax_global,InputConstMin_global,CurrentState,Reference,Hp,statePenalty,inputPenalty);
    
    % The next line overrides the controller action, comment out for step
    % response
    % U = [0;0];


    if diagnostics == 1
        error('The problem is infeasible');
        break;
    end  
    
    % Simulate NonLinear Plant ________________________
    Sim = sim('nonlinearSim');
    % Get simulation states
    h1_h2_nonlinear = Sim.h1_h2_nonlinear';
    h3_h4_nonlinear = Sim.h3_h4_nonlinear';
    % Get final States
    h1_h2_nonlinear = h1_h2_nonlinear(:,end);
    h3_h4_nonlinear = h3_h4_nonlinear(:,end);
    
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
simTime = toc(startSim);
disp(['Simulation took ',num2str(simTime), ' seconds.'])
disp(['Average controller computational time: ',num2str(mean(CalcTime))])
disp(['Max controller computational time: ',num2str(max(CalcTime))])
disp(['In ',num2str(sum(CalcTime>ts)) ,' instances, the controller computational time exceeded the sampling time.'])


% PLOT RESULTS _________________________________________________________________________________________

% Extract Data from Simulation
t   = 0:ts:timeSpan;
h1_nl  = statesHist_nl(1,:) + h1_0;
h2_nl  = statesHist_nl(2,:) + h2_0;
h1_l   = statesHist_l(1,:)  + h1_0;
h2_l   = statesHist_l(2,:)  + h2_0;
v1     = controlHist(1,:)   + u1_0;
v2     = controlHist(2,:)   + u2_0;

% Create system constraint vectors for ploting
h1_max = repmat(stateMax_global,1,length(t));
h1_min = repmat(stateMin_global,1,length(t));
h2_max = repmat(stateMax_global,1,length(t));
h2_min = repmat(stateMin_global,1,length(t));

u1_max = repmat(InputConstMax_global(1),1,length(t));
u1_min = repmat(InputConstMin_global(1),1,length(t));
u2_max = repmat(InputConstMax_global(2),1,length(t));
u2_min = repmat(InputConstMin_global(2),1,length(t));

%____________
% USE THIS BLOCK FOR STEP PLOT - IGNORE WHEN IMPLEMENTING CONTROLLER
% 
% % Plot Height of Tank 1 - IGNORE WHEN IMPLEMENTING CONTROLLER
% subplot(1,2,1)
% hold on; grid; 
% plot(t,h1_nl); plot(t,h1_l); 
% xlabel('time [s]');ylabel('states [cm]'); 
% legend('h1 plant','h1 linearised model');
% title('Tank 1 Height')
% 
% % Plot Height of Tank 2  - IGNORE WHEN IMPLEMENTING CONTROLLER
% subplot(1,2,2)
% hold on; grid; 
% plot(t,h2_nl); plot(t,h2_l); 
% xlabel('time [s]');ylabel('states [cm]'); 
% legend('h2 plant','h2 linearised model');
% title('Tank 2 Height')
%____________

figure;
% Plot Height of Tank 1
subplot(3,2,[1,3])
plot(t,h1_nl,'k','LineWidth',1.8);hold on; grid; 
plot(t,h1_l,'r','LineWidth',1.8);
plot(t,REF1,'b','LineWidth',1.8); 
plot(t,h1_max,':k','LineWidth',1.8); 
plot(t,h1_min);
xlabel('time [s]');ylabel('states [cm]'); 
legend('h1 plant','h1 controller','h1 ref','h1 max','h1 min');
title('Tank 1 Height')

% Plot Height of Tank 2
subplot(3,2,[2,4])
plot(t,h2_nl,'k','LineWidth',1.8);
hold on; grid; 
plot(t,h2_l,'r','LineWidth',1.8);
plot(t,REF2,'b','LineWidth',1.8); 
plot(t,h2_max,':k','LineWidth',1.8);
plot(t,h2_min);
xlabel('time [s]');ylabel('states [cm]'); 
legend('h2 plant','h2 controller','h2 ref','h2 max','h2 min');
title('Tank 2 Height')

% Plot control action of pump 1
subplot(3,2,5)
plot(t,v1,'k','LineWidth',1.8);
hold on; grid; 
plot(t,u1_max); plot(t,u1_min);
xlabel('time [s]');ylabel('control signal [V]'); 
legend('v1','v1 max','v1 min');
title('Pump 1 Control Signal')

% Plot control action of pump 1
subplot(3,2,6);hold on;
plot(t,v2,'k','LineWidth',1.8); grid; 
plot(t,u2_max,'k'); plot(t,u2_min,'g');
xlabel('time [s]');ylabel('control signal [V]'); 
legend('v2','v2 max','v2 min');
title('Pump 2 Control Signal')
    
% Restore User Path
userpath('reset')    
                          
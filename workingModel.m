% Initialize 
close all; clc; clear;
% Current Folder and Set new User Path;
fileName    = 'CRA1_Carrizosa-Palacios.m'; 
filePath    = matlab.desktop.editor.getActiveFilename;
proyectPath = filePath(1:length(filePath)-length(char(fileName)));
userpath(proyectPath);

% SYSTEM MODEL - SYMBOLIC __________________________________________________________________________
clc; clear;
% Variables
syms a1_ A1_ h1_        % Tank 1
syms a2_ A2_ h2_        % Tank 2
syms a3_ A3_ h3_        % Tank 3 
syms a4_ A4_ h4_        % Tank 4
syms u1_ u2_            % Inputs
syms k1_ k2_            % Pump Constants
syms gamma1_ gamma2_    % Flow Ratio
syms g_                 % Gravitational Term
syms kc_                % Sensor Constant 

% Simulation Model -  Nonlinear Model
h1_dot = -a1_/A1_*sqrt(2*g_*h1_)+a3_/A1_*sqrt(2*g_*h3_)+gamma1_*k1_*u1_/A1_;
h2_dot = -a2_/A2_*sqrt(2*g_*h2_)+a4_/A2_*sqrt(2*g_*h4_)+gamma2_*k2_*u2_/A2_;
h3_dot = -a3_/A3_*sqrt(2*g_*h3_)+(1-gamma2_)*k2_*u2_/A3_;
h4_dot = -a4_/A4_*sqrt(2*g_*h4_)+(1-gamma1_)*k1_*u1_/A4_;

% Control Model    -  Linear Model
F_         = [h1_dot; h2_dot; h3_dot; h4_dot];
X_         = [h1_, h2_, h3_, h4_];
U_         = [u1_, u2_];


A_lin     = jacobian(F_,X_);
B_lin     = jacobian(F_,U_);
C_lin     =  [kc_ 0 0 0; 0 kc_ 0 0];
D_lin     = zeros(2,2);

% SYMBOLIC EQUILIBRIUM POINTS _________________________________________________________________________

[sol_h3,sol_h4,sol_u1,sol_u2] = solve(h1_dot,h2_dot,h3_dot,h4_dot,h3_,h4_,u1_,u2_);


% SYSTEM PARAMETERS ___________________________________________________________________________________

% Tank's cross-section
A1 = 4.9;      A2 = 4.9;     A3 = 4.9;     A4 = 4.9;  % cm^2
% Tank's outlet cross-section
a1 = 0.03;     a2 = 0.03;    a3 = 0.03;    a4 = 0.03; % cm^2
% Pump constants
k1 = 1.6;      k2 = 1.6;        % cm^3*(V*s)^-1
% Flow Ratio
gamma1 = 0.3;  gamma2 = 0.3;    % cm^2
% Sensor Constant
kc = 0.5;                       % V*cm^-1
% Gravitational Constant
g  = 981;                       % cm/s^2
 

% REALIZATION OF SYSTEM AND EQUILIBRIUM POINTS __________________________________________________________

% Equilibrium States (cm)   - (DEFINE HERE)
h1_0 = 10;
h2_0 = 10;

% Discretization sampling time
ts = 1;

% Dependent Equilibrium States (cm)
h3_0  = double(subs(sol_h3,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_], ...
                           [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0]));
h4_0  = double(subs(sol_h4,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_], ...
                           [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0]));   
% Dependent Equilibrium Inputs (V)
u1_0 = double(subs(sol_u1,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_], ...
                           [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0]));
u2_0 = double(subs(sol_u2,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_], ...
                           [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0]));

EquilibriumPoints = [h1_0, h2_0, h3_0, h4_0];            % LATER PASSED TO MPC
EquilibriumInputs = [u1_0, u2_0];                        % LATER PASSED TO MPC

% Verify Equilibrium
% double(subs(h1_dot,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
%                            [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0 h4_0 u1_0 u2_0]))
%                     
% double(subs(h2_dot,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
%                            [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0 h4_0 u1_0 u2_0])) 
%                        
% double(subs(h3_dot,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
%                            [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0 h4_0 u1_0 u2_0]))
%                     
% double(subs(h4_dot,[A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
%                            [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0 h4_0 u1_0 u2_0]))                      

       
% System Matrices
A_lin = double(subs(A_lin, [A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
                             [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0,h4_0 u1_0 u2_0]));
B_lin = double(subs(B_lin, [A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
                             [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0,h4_0 u1_0 u2_0]));
C_lin = double(subs(C_lin, [A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
                             [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0,h4_0 u1_0 u2_0]));                         
D_lin = double(subs(D_lin, [A1_ A2_ A3_ A4_ a1_ a2_ a3_ a4_ k1_ k2_ gamma1_ gamma2_ kc_ g_ h1_ h2_ h3_ h4_ u1_ u2_], ...
                             [A1 A2 A3 A4 a1 a2 a3 a4 k1 k2 gamma1 gamma2 kc g h1_0,h2_0 h3_0,h4_0 u1_0 u2_0]));
                         
% Define continous time state space model
SYS_lin_ct = ss(A_lin,B_lin,C_lin,D_lin);        

% Discretize model
SYS_lin_dt = c2d(SYS_lin_ct,ts,'tustin');                 % LATER PASSED TO MPC 
A = SYS_lin_dt.A;
B = SYS_lin_dt.B;
C = SYS_lin_dt.C;
D = SYS_lin_dt.D;
%                                     
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
    [U,calcTime, diagnostics] = MPC_fun2(SYS_lin_dt,EquilibriumPoints,EquilibriumInputs, StateConstMax_global,...
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
                          
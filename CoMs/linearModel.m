global A B C D Bd SlewRate StatesMin StatesMax normalX normU uMax uMin InputsMax InputsMin Ref

%Parameter b
b1 = 1/(capacity*bat_nom_V);   
b2 = -1/(capacity*bat_nom_V);

% MODEL:
A  = [1 0.0278*Ts;0 0];
B  = [0;b1];
Bd = [0;b2];
C  = eye(2);
D  = 0;

% References
Ref = [socRef;crateRef];

% Constraints defined in main
StatesMin = [SoC_Min; C_rate_Min];   
StatesMax = [SoC_Max; C_rate_Max];

% Constraints on control signal
InputsMax = uMax;
InputsMin = uMin;

% Model input slew rate
SlewRate  = 0.1*Iend*Vend*Ts;

% Normalization of states and inpute for controller
normalX = [30; 1];
normU   = 20;


clc
syms  i_fc
R       = 8.3145;           % GAS CONSTANT [J/(mol K)]
F       = 96485;            % A s/mol
z       = 2;                % Number of moving electrons
En_cell = 1.1632;           % Nerst Voltage per cell [V]
nCells  = 20;               % Number of cells
En      = En_cell*nCells;   % Stack Nernst voltage [V]
alpha   = 0.18409;          % Charge transfer coefficient
k       = 1.38e-23;         % Boltzmann's constant [J/K]
h       = 6.626e-34;        % Planck's constant [J s]
delVol  = 1;                % Activation barrier volume factor [m3]. The size of activation barrier (ΔG) is computed assuming Δv = 1m3
% delG  =                   % Size of the activation barrier [J/mol]
T       = 298;              % Temperature of operation [K]
% Kc    =                   % Voltage constant at nominal condition of operation
A       = R*T/(z*alpha*F);  % Tafel slope

Pfuel   = 1.345;            % Absolute fuel supply pressure [atm]
Pair    = 1.296;            % Absolute air supply pressure  [atm]
Vfuel   = 1.885;            % Fuel flow rate [l/min]                  !!! (NOMINAL, think about obtaining signal for our model)
Vair    = 20;               % Air flow rate [l/min]                   !!! (NOMINAL, think about obtaining signal for our model)
x       = 99.95;            % Percentage of hydrogen in fuel
y       = 21;               % Percentage of oxygen in oxidant

%
U_fh2   = 60000*R*T*i_fc/(z*F*Vfuel*Pfuel*x); % Rates of conversion / Utilization of hydrogen
U_fo2   = 60000*R*T*i_fc/(2*z*F*Vair*Pair*y); % Rates of conversion / Utilization of oxygen

PH2     = (1-U_fh2)*x*Pfuel;                  % Partial pressure of hydrogen inside the stack [Pa]
PO2     = (1-U_fo2)*y*Pair;                   % Partial pressure of oxygen inside the stack [Pa]


En      = 1.229 + (T-298)*-44.43/(z*F)+R*T/(z*F)*log(PH2*sqrt(PO2));

i   = 0.00001:5;
V   = subs(En,i_fc,i);
V   = zeros(1,length(i));
idx = 1;
for iFc = i
    V(idx) = subs(En,i_fc,iFc);
    idx=idx+1;
end
plot(i,V)

% 1 
Eoc     = 18;    % Voltage at 0 Ampere output [V]
V1      = 15;    % Voltage at 1 Ampere output [V]

Vnom    = 12.5;  % Nominal Voltage            [V]
Inom    = 4;     % Nominal Current            [A]

Imax    = 5;     % Maximum operating current  [A]
Vmin    = 12;    % Maximum operating voltage  [V]

% 2
nCells  = 20;               % Number of cells
etaEFF  = 0.46;             % Nominal Stack Efficiency
T       = 298;              % Temperature of operation [K]


Eoc     = 18;               % Open circle voltage
A       = R*T/(z*alpha*F);  % Tafel slope
Rohm    = 0.18872;          % 0.18872;

%%

Eoc     = 18;               % Open circuit voltage
nCells  = 20;               % Number of cells

R       = 8.3145;           % GAS CONSTANT [J/(mol K)]
T       = 298;              % Temperature of operation [K]
z       = 2;                % Number of moving electrons
alpha   = 0.18409;          % Charge transfer coefficient
F       = 96485;            % A s/mol
A       = R*T/(z*alpha*F);  % Tafel slope

io      = 0.13328;          % Exchange current
s       = tf('s');




R       = 8.3145;           % GAS CONSTANT [J/(mol K)]
F       = 96485;            % A s/mol
z       = 2;                % Number of moving electrons
En_cell = 1.1632;           % Nerst Voltage per cell [V]
nCells  = 20;               % Number of cells
En      = En_cell*nCells;   % Stack Nernst voltage [V]
alpha   = 0.18409;          % Charge transfer coefficient
k       = 1.38e-23;         % Boltzmann's constant [J/K]
h       = 6.626e-34;        % Planck's constant [J s]
delVol  = 1;                % Activation barrier volume factor [m3]. The size of activation barrier (ΔG) is computed assuming Δv = 1m3
% delG  =                   % Size of the activation barrier [J/mol]
T       = 298;              % Temperature of operation [K]
% Kc    =                   % Voltage constant at nominal condition of operation
A       = R*T/(z*alpha*F);  % Tafel slope

Pfuel   = 1.345;            % Absolute fuel supply pressure [atm]
Pair    = 1.296;            % Absolute air supply pressure  [atm]
Vfuel   = 1.885;            % Fuel flow rate [l/min]                  !!! (NOMINAL, think about obtaining signal for our model)
Vair    = 20;               % Air flow rate [l/min]                   !!! (NOMINAL, think about obtaining signal for our model)
x       = 99.95;            % Percentage of hydrogen in fuel
y       = 21;               % Percentage of oxygen in oxidant


%%
close all;
fig = gcf;
Eoc     = 18;               % Open circuit voltage

R       = 8.3145;           % GAS CONSTANT [J/(mol K)]
T       = 298;              % Temperature of operation [K]
z       = 2;                % Number of moving electrons
alpha   = 0.18409;          % Charge transfer coefficient
F       = 96485;            % A s/mol
A       = R*T/(z*alpha*F);  % Tafel slope

nCells  = 20;               % Number of cells
Rohm    = 0.18872;          % 0.18872;

i       = 0:0.05:5;
io      = 0.13328;          % Exchange current
i_plot  = i; i_plot(i_plot<io) = io;

Imax    = 5;                % Maximum operating current  [A]
io      = 0.13328;          % Exchange current

m = 1e-2;                   % [V]
n = 0.330;                  % [cm^2/A]

Eoc     = 18;
coeff   = 1e-8;

E = Eoc -nCells*A.*log(i_plot./io)-Rohm.*i; %- nCells*m*exp(n*i_plot);
P = E.*i./1000; % In Kw
fig
subplot(2,1,1); hold on;
stem(i,E); 
legend
subplot(2,1,2); hold on;
stem(i,P); 
legend

%%

E_0 = 46.5595;
K   = 0.0027639;
A   = 43.2154;
B   = 0.108087;
Q   = 2.2;
I   = I_bat_hist.Data;
it  = cumsum(I).*1/Q;
V   = E_0 - K.*Q./(Q-it).*it+A.*exp(-B.*it);

plot(V_bat_hist,'k','LineWidth',1.8);





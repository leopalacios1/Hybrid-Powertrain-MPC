% Constraint on input and states;
SoC_Max    = 80;   % Percent
SoC_Min    = 20;   % Percent
C_rate_Max = inf;  % h^-1
C_rate_Min = -inf; % h^-1
uMax       = 50;   % Watts
uMin       = 20;    % Watts

StatesMin = [SoC_Min; C_rate_Min];
StatesMax = [SoC_Max; C_rate_Max];

InputsMin = [uMin];
InputsMax = [uMax];



% Parameter b
Bat_Q = 2.2;             %Ah
simTime = 500;

sim('SA_50')
f = figure('visible','on');

V_fc_mean    = mean(V_fc_hist.Data(63423:end))
V_bat_mean   = mean(V_bat_hist.Data(63423:end))
P_fc_mean    = mean(P_fc_hist.Data(63423:end))
P_load_mean  = mean(P_load_hist.Data(63423:end))
I_boost_mean = mean(I_boost_hist.Data(63423:end))
I_bat_mean   = mean(I_bat_hist.Data(63423:end))
I_fc_mean    = mean(I_fc_hist.Data(63423:end))
Volt_across_mean = mean(Volt_across_hist.Data(63423:end))


plot(V_fc_hist,'k','LineWidth',1.8);hold on; grid; 
plot(V_bat_hist,'r','LineWidth',1.8);
lg=legend('V_fc_hist','V_bat_hist');
lg.FontSize=8;
title('Voltages')


% Plot Powers
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


% CoM

Ts    = 0.01;
V_fc  = 14.3442;
V_bat = 43.0420;

Bat_Q = 2.2;                 %Ah
b1    = 1/(Bat_Q*V_bat);      %Watt*h   [REMOVED 3600*]
b2    = -1/(Bat_Q*V_bat);

A  = [1 0.0278*Ts; 0 0]; % 0.0278 = 100/3600
B  = [0;b1];
Bd = [0;b2];

X      = zeros(2,length(0:Ts:simTime));
X(:,1) = [50 ,0];
k      = 1;

P_load = 25.8252;
P_cell = 21.4665;



for t = Ts:Ts:simTime
    X(:,k+1)  = A*X(:,k) + B*P_cell + Bd*P_load;
    k = k+1;
end

SoC  = X(1,:);
Time = 0:Ts:simTime;
%%
figure;
plot(SOC_hist,'k','LineWidth',1.8); grid; hold on;
plot(Time,SoC,'r','LineWidth',1.8)
lg=legend('SoM - SoC','CoM - SoC');
lg.FontSize=8;
title('SoM vs. CoM')
%%
IDX = zeros(1,length(0:Ts:simTime));
k=1;
for t = 0:Ts:simTime
    idx = find(SOC_hist.Time>=t,1);
    IDX(k) = idx;
    k=k+1;
end
%%
Adj_SoC_hist = SOC_hist.Data(IDX)';

errorEvol = (-Adj_SoC_hist + SoC)./Adj_SoC_hist*100;
figure;
plot(Time,errorEvol,'k','LineWidth',1.8); grid;
lg=legend('Model Error');
lg.FontSize=8;
title('SoM vs. CoM - Error Evolution ')


clc; clear; close all;
global Ts SoC_Max SoC_Min C_rate_Max C_rate_Min uMax uMin StatesMin StatesMax  InputsMin InputsMax Hp;


filePath    = matlab.desktop.editor.getActiveFilename;
fileName    = filePath(find(filePath=='/',1,'last')+1:end);
proyectPath = filePath(1:length(filePath)-length(char(fileName))); 
userpath(proyectPath);
cd(proyectPath)


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

simTime = 50;
Ts      = 1;
Hp      = 25;


tic
% for Ts = [1,10:10:30]
%     for Hp = [10:15:70]
        
        fileName = ['Ts_',num2str(Ts),'__','Hp_',num2str(Hp),'.png'];
        sim('SA_50')
        f = figure('visible','on');

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
toc
%         cd TEST/
%         saveas(f,fileName)
%         cd ..
%     disp(['Done with: Ts_',num2str(Ts),'__','Hp_',num2str(Hp),'.png'])
%     end
% end


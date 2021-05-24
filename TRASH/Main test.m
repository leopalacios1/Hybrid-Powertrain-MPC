%clc; clear; close all;
close all; clc
global Ts SoC_Max SoC_Min C_rate_Max C_rate_Min uMax uMin StatesMin StatesMax  InputsMin InputsMax Hp Q R;


filePath    = matlab.desktop.editor.getActiveFilename;
fileName    = filePath(find(filePath=='/',1,'last')+1:end);
proyectPath = filePath(1:length(filePath)-length(char(fileName)))
userpath(proyectPath);
addpath([proyectPath,'/MPCs']); % Add path for simulink model to find controller function.
addpath([proyectPath,'/PARAMETERS']);
cd(proyectPath)

parameterConfig1

% Constraint on input and states;
SoC_Max    = 80;   % Percent
SoC_Min    = 20;   % Percent
C_rate_Max = inf;  % h^-1
C_rate_Min = -inf; % h^-1
uMax       = 50;   % Watts
uMin       = 20;   % Watts

StatesMin = [SoC_Min; C_rate_Min ; -inf; -inf];   
StatesMax = [SoC_Max; C_rate_Max;  inf;   inf];

InputsMin = [uMin];
InputsMax = [uMax];

% Distrubance
powerProfile = 3;

% Parameter b
Bat_Q = 2.2;             %Ah

simTime = 15;
Ts      = 1;


i = 1;

for Hp = 25%[20,30]
  for q_soc = 20%[80,500]
      for q_crate = 600%[1,80,500]
          for r = 400%[80,500]
                Q=diag([q_soc q_crate 0 0]);   % penalty on states
                R=diag([r]); 
                fileName   = ['HP',num2str(Hp),'_Qa',num2str(q_soc),'_Qb',num2str(q_crate),'_R',num2str(r),'.png'];
                SimResults =  sim('SA_50_SIMPLE_MOSFET')
                f(i) = figure('visible','on');

                C_rate_hist     = SimResults.C_rate_hist;   
                P_fc_hist       = SimResults.P_fc_hist;     
                P_fc_ref_hist   = SimResults.P_fc_ref_hist; 
                P_load_hist     = SimResults.P_load_hist;   
                SOC_hist        = SimResults.SOC_hist;      
                time            = SimResults.time;          
                tout            = SimResults.tout;         

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
                cd ..
                cd RESULTS/
                saveas(f,fileName)
                cd ..
                cd SoMs
                disp(['Done with: Ts_',fileName])
                i=i+1;
                
          end
          
      end
      
  end
  
end



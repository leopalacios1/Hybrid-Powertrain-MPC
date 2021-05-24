 function [powerFC,compTime] =  slow_linearMPC(SoC,C_rate,V_bat,P_load,V_fc, time,lastPfc)

        global Ts SoC_Max SoC_Min C_rate_Max C_rate_Min uMax uMin StatesMin StatesMax  InputsMin InputsMax Hp Q R;
        
if time > 0.000000001  
     
        %Current State
        States = [SoC,C_rate,lastPfc,0];
        %Hp = 50;

        %MODEL PARAMETERS  
        %HARDWIRED FOR TESTING - CHANGE FOR FLEXIBILITY
        %____________________________________________________
        %____________________________________________________

       % Constraint on input and states;
%         SoC_Max    = 80;   % Percent
%         SoC_Min    = 20;   % Percent
%         C_rate_Max = inf;  % h^-1
%         C_rate_Min = -inf; % h^-1
%         uMax       = 60;   % Watts
%         uMin       = 20;    % Watts
%         
%         StatesMin = [SoC_Min; C_rate_Min];
%         StatesMax = [SoC_Max; C_rate_Max];
% 
%         InputsMin = [uMin];
%         InputsMax = [uMax];
%         
%         disp('____________________________________________________________________________________' )
%         disp(['The controller was called at time ',num2str(time),' with the following parameters: '])
%         disp(['SoC: ',num2str(SoC)])
%         disp(['C_rate: ',num2str(C_rate)])
%         disp(['V_bat: ',num2str(V_bat)])
%         disp(['V_fc: ',num2str(V_fc)])
%         disp(['P_load: ',num2str(P_load)])
%         disp('----' )


        %Parameter b
        Bat_Q = 2.2;               % Ah
        b1 = 1/(Bat_Q*V_bat);      % Watt*h   [REMOVED 3600*]   !!!! CHANGED V_fc FOR V_bat
        b2 = -1/(Bat_Q*V_bat);

       % References
        r = [50;0;0;0];
        ts = 1;
       % MODEL:
        A  = [1 0.0278*Ts 0 0; 0 0 0 0; 0 0 0 0;0 0 1 0];
        B  = [0;b1;1;-1];
        Bd = [0;b2;0;0];
        C  = eye(4);
        D  = 0;
        
        
        %Weighting Matrices
       % Q=diag([10 300 0 0]);   % penalty on states

       % R=diag([50]);     % penalty on control action

        %____________________________________________________
        %____________________________________________________
       
        %MPC PARAMETERS ____________________________________________________________
        nx= size(A,1);   % number of states
        nu= size(B,2) ;  % number of input

        tic % INITIALIZE TIME CLOCK
        
        %Symbolic variable declaration
        u  = sdpvar(repmat(nu,1,Hp), ones(1,Hp));
        x0 = sdpvar(nx,1);
        x  = x0;
        
        %CHANGE LATER
        normalX = [50; 1;1;1];
        normU   = 50;
        constraints = [];
        objective   = 0;
        for j = 1:Hp
            if isnan(P_load)  || P_load == 0
              x = A*x + B*u{j};  % + Bd*P_load
              disp('GOT HERE')
            else
              x = A*x + B*u{j} + Bd*P_load;
            end
            objective = objective + ((r-x(1:4))./normalX)'*Q*((r-x(1:4))./normalX) + u{j}./normU'*R*u{j}./normU;
            constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                      StatesMin <=   x  <= StatesMax];
        end

        controller = optimizer(constraints,objective,sdpsettings('solver','quadprog'),x0,u);

        x0 = States; % initial condition for states
        x  = x0';
       try
          [u,diagnostics] = controller{x};
       catch
           disp('The problem is infeasible or some other error occured, go figure');
       end
       
        if ~isnan(u{1})
         powerFC = u{1};
        else 
         powerFC = 0;
        end
%         disp(['Reference FC power obtained: ',num2str(powerFC)])
%         disp(['With a corresponding Ifc: ',num2str(powerFC/V_fc)])
else
    powerFC = 0;
end
end
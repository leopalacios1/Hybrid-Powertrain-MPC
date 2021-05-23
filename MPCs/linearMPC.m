function [powerFC,compTime] =  linearMPC(SoC,C_rate,V_bat,P_load,V_fc, time)

 % DEFINE GLOBAL VARIABLES
 global  StatesMin StatesMax  InputsMin InputsMax Hp Q R  A B Bd SlewRate Ref normalX normU
 
 % IF DONE FOR AVOIDING ERRORS, FIND A BETTER WAY.
if time > 0.000000001 
    
        % Clear YALMIP variables                            
        yalmip('clear')   
        
        % Current State
        States = [SoC,C_rate];

        % MPC PARAMETERS ____________________________________________________________
        nx= size(A,1);   % number of states
        nu= size(B,2);   % number of input

        tic % INITIALIZE TIME CLOCK

        %Symbolic variable declaration
        u  = sdpvar(repmat(nu,1,Hp), ones(1,Hp));
        x0 = sdpvar(nx,1);
        x  = x0;

        constraints = [];
        objective   = 0;

        for j = 1:Hp
            if isnan(P_load)  || P_load == 0
              x = A*x + B*u{j};  % + Bd*P_load
              disp('GOT HERE')
            else
              x = A*x + B*u{j} + Bd*P_load;
            end
            objective =objective + ((Ref-x)./normalX)'*Q*((Ref-x)./normalX) + u{j}./normU'*R*u{j}./normU;% objective + (r-x(1:2))'*Q*(r-x(1:2)) + u{j}'*R*u{j};
            if j>1
                constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                          StatesMin <=   x  <= StatesMax , ...
                                          -SlewRate <= u{j} - u{j-1}<= SlewRate];
            else
                constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                          StatesMin <=   x  <= StatesMax];
            end
        end

        controller = optimizer(constraints,objective,sdpsettings('solver','quadprog'),x0,u);

        x0 = States; % initial condition for states
        x  = x0';
        
        try 
          [u,diagnostics] = controller{x};
        catch
           disp('The problem is infeasible or some other error occured, go figure')
        end
         powerFC = u{1};
%         disp(['Reference FC power obtained: ',num2str(powerFC)])
%         disp(['With a corresponding Ifc: ',num2str(powerFC/V_fc)])
else
    powerFC = 0;
end
end

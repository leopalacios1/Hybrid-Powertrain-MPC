function [OUT] =  linearMPC(SoC,C_rate,V_bat,P_load,V_fc, time)

 % DEFINE GLOBAL VARIABLES
 global  controller
 
 % IF STATEMENT IMPLEMENTED FOR AVOIDING ERRORS, FIND A BETTER WAY.
if time > 0.000000001   
        % Clear YALMIP variables                            
        yalmip('clear')        
        % NOTE: CONTROLLER DEFINITION CAN BE FOUND ON PROJECT/SoMs/linearMPCconfig
        
        x  = [SoC;C_rate];   % initial condition for states 
        tic 
        try 
          [u,diagnostics] = controller(x,P_load);
        catch
           disp('The problem is infeasible or some other error occured, go figure')
        end
        compTime = toc;
        powerFC = u{1};
        OUT = [powerFC;compTime];
else
    powerFC  = 0;
    compTime = 0;
    OUT = [powerFC;compTime];
end
end

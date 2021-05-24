function [OUT] =  linearMPC(SoC,C_rate,V_bat,P_load,V_fc, time)

 % DEFINE GLOBAL VARIABLES
 global  controller lastPfc  controller2 uMin
     if time > 0.000000001  
         if lastPfc > uMin
         % IF STATEMENT IMPLEMENTED FOR AVOIDING ERRORS, FIND A BETTER WAY.

                    % Clear YALMIP variables                            
                    yalmip('clear')        
                    % NOTE: CONTROLLER DEFINITION CAN BE FOUND ON PROJECT/SoMs/linearMPCconfig

                    x  = [SoC;C_rate];   % initial condition for states 
                    tic 
                    try 
                      [u,diagnostics] = controller(x,P_load,lastPfc);
                    catch
                       disp('The problem is infeasible or some other error occured, go figure')
                    end
                    compTime = toc;
                    powerFC  = u{1};
                    lastPfc  = powerFC;
                    OUT = [powerFC;compTime];
         else 
                    % Clear YALMIP variables                            
                    yalmip('clear')        
                    % NOTE: CONTROLLER DEFINITION CAN BE FOUND ON PROJECT/SoMs/linearMPCconfig

                    x  = [SoC;C_rate];   % initial condition for states 
                    tic 
                    try 
                      [u,diagnostics] = controller2(x,P_load,lastPfc);
                    catch
                       disp('The problem is infeasible or some other error occured, go figure')
                    end
                    compTime = toc;
                    powerFC  = u{1};
                    lastPfc  = powerFC;
                    OUT = [powerFC;compTime];

         end

     else
            powerFC  = 0;
            compTime = 0;
            OUT = [powerFC;compTime];
     end
end

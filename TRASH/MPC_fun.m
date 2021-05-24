function [nextControl,time,diagnostics] =  MPC_fun(Model,EquilibriumPoints,EquilibriumInputs, StateConstMax_global,StateConstMin_global, ...
                                      InputConstMax_global,InputConstMin_global,CurrentState,Reference,Hp,statePenalty,inputPenalty)
                                                             
    % Clear YALMIP variables                            
    yalmip('clear')   
    
    % INITIALIZATION OF CONSTRAINTS ___________________________________________
    
    % Equilibrium States
    h10   = EquilibriumPoints(1);
    h20   = EquilibriumPoints(2);
    h30   = EquilibriumPoints(3); 
    h40   = EquilibriumPoints(4);
    
    % Equilibrium Inputs
    u10   = EquilibriumInputs(1);
    u20   = EquilibriumInputs(2);
    
    % Shift global constraints relative to equilibrium points.
    hMax1 = StateConstMax_global(1)-h10;
    hMin1 = StateConstMin_global(1)-h10;
    hMax2 = StateConstMax_global(2)-h20;
    hMin2 = StateConstMin_global(2)-h20;
    hMax3 = StateConstMax_global(3)-h30;
    hMin3 = StateConstMin_global(3)-h30;
    hMax4 = StateConstMax_global(4)-h40;
    hMin4 = StateConstMin_global(4)-h40;
    
    uMax1 = InputConstMax_global(1) - u10;
    uMax2 = InputConstMax_global(1) - u20;
    uMin1 = InputConstMin_global(1) - u10;
    uMin2 = InputConstMin_global(2) - u20;
    
    % Current State
    States = CurrentState(1:4);
    % Reference
    r = [Reference(1)-h10, Reference(2)-h20];
    r = r';
    
    % DISCRETIZED MODEL
    A = Model.A;
    B = Model.B;

       
   % MPC PARAMETERS ____________________________________________________________

    nx= size(A,1);   % number of states
    nu= size(B,2);   % number of inputs
    
    % Weighting Matrices
    Q=diag(statePenalty);   % penalty on states
    R=diag(inputPenalty);   % penalty on control action
    
    
    tic % INITIALIZE TIME CLOCK
    %Symbolic variable declaration
    u  = sdpvar(repmat(nu,1,Hp), ones(1,Hp));
    x0 = sdpvar(nx,1);
    x  = x0;
    
    constraints = [];
    objective   = 0;

    for j = 1:Hp
        x = A*x + B*u{j};
        objective = objective + (r-x(1:2))'*Q*(r-x(1:2)) + u{j}'*R*u{j};
        constraints=[constraints, [uMin1,uMin2]' <= u{j} <=[uMax1 uMax2]',...
                    [hMin1 hMin2 hMin3 hMin4]'   <= x    <=[hMax1 hMax2 hMax3 hMax4]];
    end
    
    controller = optimizer(constraints,objective,sdpsettings('solver','quadprog'),x0,u);
    
    x0 = States; % initial condition for states
    x  = x0;

    [u,diagnostics] = controller{x};
    
    if diagnostics == 1
        error('The problem is infeasible');
    end  
    
    nextControl = u{1};
    time = toc;
end
clc; clear all;
import casadi.*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function [powerFC,compTime] =  nonLinearMPC(SoC,C_rate,V_bat,P_load,V_fc, time,lastPfc)

    %global  StatesMin StatesMax  InputsMin InputsMax Hp Q R Ts  Bd SlewRate Ref normalX normU controller SoC_Min SoC_Max C_rate_Min C_rate_Max
    
    % for testing ____________________________________________________________
            
            normalX = [30; 1; 1];
            normU   = 20;
            
            % State References

            socRef     = 50;        % State of charge reference             [%]
            crateRef   = 0;         % C-rate reference                      [h^-1]

            % Constraint on input and states;
            SoC_Max    = 80;        % Maximum State of Charge               [%]
            SoC_Min    = 20;        % Minimum State of Charge               [%]
            C_rate_Max = inf;       % Maximim C-rate                        [h^-1]
            C_rate_Min = -inf;      % Minumum C-rate                        [h^-1]

            % Controller Constraints
            uMax       = 60;        % Maximum allowed FC power              [W]
            uMin       = 20;        % Minimum allowed FC power              [W]

            % Sampling time and Prediction Horizon Definition
            TsVec      = 1  ;     % Controller Sampling Times               [s]
            HpVec      = 30 ;     % Prediction Horizon Sampling Times       [#]

            % Constraints VEC
            StatesMin = [SoC_Min; C_rate_Min];   
            StatesMax = [SoC_Max; C_rate_Max];
            
            % Constraints on control signal
            InputsMax = uMax;
            InputsMin = uMin;
            
            % Controller Setting
            Ts        = 1;   % [s]
            Hp        = 25;  % [#]
            
            % Current State of system (for testing)
            Crate_state = 0.5;
            SoC_state   = 40;
            Pload_now   = 55;
            SR = 6;
            actual_SOC  = SoC_state;
            actual_Crate = Crate_state;
            actual_U    = Pload_now;
    
    % for testing ____________________________________________________________
    
    
    % CONTROLLER SETUP

    % Declare model variables
    x1 = SX.sym('x1',1);       % SoC
    x2 = SX.sym('x2',1);       % C-rate
    u = SX.sym('u',1);         % P-fc
    du = SX.sym('du',1);         % P-fc
    %d = SX.sym('d',1);         % P-load
    x = [x1; x2;u];              % State Vec
    d = Pload_now;
    %=============MODIFICATIONS SETUP HERE =====================
    % Controller configuration
    dt     = Ts;                    %Sampling period [s]
    min_Uk = -SR;
    min_X3 = InputsMin;
    max_Uk = SR;
    max_X3 = InputsMax;
    min_X1 = SoC_Min;
    max_X1 = SoC_Max;
    min_X2 = C_rate_Min;
    max_X2 = C_rate_Max;

    % Nonlinear equation parameters

    E_0 = 46.5595;
    K   = 0.13621;
    A   = 43.2154;
    B   = 0.108087;
    Qbat   = 2.2;
    R   = 0.18182;
    I   = x(2)*Qbat;      %3600*x(2)*Q;      % THIS ONE (?)    -> C-rate = I /Q   (MEJOR PONER EL 3600 y Ts en la expresion del SoC.
    it  = Qbat*(1-x(1)/100);
    V   = E_0 - R*I- K.*Qbat./(Qbat-it).*it+A.*exp(-B.*it);
    % HASTA AQUI HE CAMBIADO, LO QUE VIENE HAY QUE IR EDITANDO
    
    
    
    
    %Number of control intervals to analyse
    N = Hp;
    % Define model equations
%     x_next = [x(1) + dt*((q/V)*(Caf-x(1))-ko*x(1)*exp(-E_R/x(2))); %calculate the theoretical next
%               x(2) + dt*((q/V)*(Tf-x(2)) +(delta_H*ko*x(1))/(rho*Cp)*exp(-E_R/x(2)) + UA/(rho*Cp*V)*(u-x(2)))];


    x_next = [x(1) + 100*dt*x(2)/3600;(x(3)+du-d)/V;x(3)+du];      
    %======================================================


    % Weigth matrices QR 
    Q = [1,0,0;0,0,0;0,0,0];
    R = 0;

    n_states = length(x);
    n_controls = length(du);


    f = Function('f',{x,du},{x_next});

    % Define prediction horizon and parameters
    P = MX.sym('P',n_states, 2); % We include the initial conditions and the point to stabilize


    % Start with an empty NLP
    w   = []; %variables to optimize
    lbw = []; %lower bounds var. to optimize
    ubw = []; %upper bounds var. to optimize
    obj = 0;  %objective function
    g   = []; %constraints
    lbg = []; %lower bounds constraints
    ubg = []; %upper bounds constraints

    % Initial conditions
    Xk = P(:,1); %the first column of the parameters is the initial condition

    % Loop in the prediction horizon
    for k = 1:N 
        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str(k)],n_controls);
        w = [w; Uk];
        lbw = [lbw; min_Uk];
        ubw = [ubw; max_Uk];

        % Integrate till the end of the interval
        Xk_next = f(Xk,Uk);
        g = [g; Xk_next];
        lbg = [lbg; min_X1; min_X2; min_X3];
        ubg = [ubg; max_X1; max_X2; max_X3];

        % Update objective function
        %obj = obj + (Xk_next-P(:,2))'*Q*(Xk_next-P(:,2)) + Uk'*R*Uk ; 
        obj = obj + ((Xk_next-P(:,2))./normalX)'*Q*((Xk_next-P(:,2))./normalX) + ((Uk+Xk_next(3))./normU)'*R*((Uk+Xk_next(3))./normU) ; 

        Xk = Xk_next; %update the initial condition for the next iteration
    end


    % We encapsulate the nlp problem in the horizon prediction into a struct
    % f funcion objetivo
    % w son las entradas
    % g son las x
    % P condicion inicial 
    nlp_prob = struct('f', obj, 'x', w, 'g', g, 'p', P);

    opts = struct;
    opts.ipopt.warm_start_init_point = 'yes';
    solver = nlpsol('solver', 'ipopt', nlp_prob,opts); %solving with IPOPT
    % solver = qpsol('solver', 'qpoases', nlp_prob); %solving with QPOASES

    %----------------------------------------------

    % FOR SIMULATION JUST MODIFY THE REFERENCE, T, AND INITIAL COND.
    %============= SIMULATION - MODIFICATIONS HERE ==========================
    T       = 20; %final time [s]
    nT      = T/dt; %number of samples
    x_ini   = [actual_SOC; actual_Crate;actual_U]; %initial condition
    store_x = [x_ini];
    store_du = [];
    store_u = [];
    for i=1:nT
        reference(:,i) = [100; 0;0]; 
    end     
    %========================================================================
    
    lbg
    ubg
    
    for i=1:nT
        args.x0  = repmat(0,N,1); %initial guess for the controls
        args.p = [x_ini reference(:,i)]; 

        % Solve the problem
        sol = solver('x0', args.x0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', args.p);

        % Get controls from the solution
        u = reshape(full(sol.x),n_controls,N)'; 

        next_x = f(x_ini,u(1,:)');

        x_ini = next_x; %update the initial conditions
        % Store things
        store_x = [store_x full(next_x)];
        store_du = [store_du u(1,:)'];
        store_u = [store_u, store_du(i)+store_x(3,i)];
    end

store_du
clc;

SoC    = 60;
C_rate = 0;

% Parameter b
Bat_Q = 2.2             %Ah
V_bat = 43
b = 1/(Bat_Q*V_bat)      %Watt*h   [REMOVED 3600*]

currentState = [SoC,C_rate]
ts = 0.001;

A  = [1 100*ts; 0 0]
B  = [0;b]
Bd = [0;-b]
C  = eye(2);
D  = 0;

P_load = 50;


u  = sdpvar(repmat(1,1,Hp), ones(1,Hp))
x0 = sdpvar(2,1)
x  = x0;

Hp = 25;

constraints = [];
objective   = 0;

r = [50;0]
Q = diag([1 1])   % penalty on states
R = diag([1])

StatesMin = [0; -inf];
StatesMax = [100; inf];
    
InputsMin = [0];
InputsMax = [60];
    
% currentStateT = currentState'
% for  i = 1:1000
%     disp([num2str(i),':'])
%     currentStateT = A*currentStateT + B*40 + Bd*P_load
%     disp(' ')
%     disp(' ')
% end


for j = 1:Hp    
    x = A*x + B*u{j} + Bd*P_load;  
    objective   = objective + (r-x(1:2))'*Q*(r-x(1:2)) + u{j}'*R*u{j};
    constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                  StatesMin <=   x  <= StatesMax]       
end


controller = optimizer(constraints,objective,sdpsettings('solver','quadprog'),x0,u)

x0 = States; % initial condition for states
x  = x0'
whos
[u,diagnostics] = controller{x}
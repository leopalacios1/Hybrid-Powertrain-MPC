% DEFINE GLOBAL VARIABLES
global  StatesMin StatesMax  InputsMin InputsMax Hp Q R  A B Bd SlewRate Ref normalX normU controller

nx= size(A,1);   % number of states
nu= size(B,2);   % number of input

%Symbolic variable declaration
u  = sdpvar(repmat(nu,1,Hp), ones(1,Hp));
x0 = sdpvar(nx,1);
P_load = sdpvar(1,1);
x  = x0;

constraints = [];
objective   = 0; 

for j = 1:Hp
      x = A*x + B*u{j} + Bd*P_load;
    objective =objective + ((Ref-x)./normalX)'*Q*((Ref-x)./normalX) + u{j}./normU'*R*u{j}./normU;% objective + (r-x(1:2))'*Q*(r-x(1:2)) + u{j}'*R*u{j};
    if j>1
        constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                  StatesMin <=   x  <= StatesMax , ...
                                  -SlewRate <= u{j} - u{j-1} <= SlewRate];
    else
        constraints=[constraints, InputsMin <= u{j} <= InputsMax , ...
                                  StatesMin <=   x  <= StatesMax];
    end
end

controller = optimizer(constraints,objective,sdpsettings('solver','quadprog'),{x0,P_load},u);


 % Simulate NonLinear Plant ________________________
 
    % Inputs
 
    Sim = sim('nonlinearSim');
    
    
    % Outputs
    
    % Get simulation states
    h1_h2_nonlinear = Sim.h1_h2_nonlinear';
    h3_h4_nonlinear = Sim.h3_h4_nonlinear';
    % Get final States
    h1_h2_nonlinear = h1_h2_nonlinear(:,end);
    h3_h4_nonlinear = h3_h4_nonlinear(:,end);
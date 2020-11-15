function [A,B] = MBPE_MPC_DC_motor_conduct_MBPE(x,u,T_s)
    
    % assign data as system identification data
    data = iddata(x, u, T_s, 'Name', 'DC-motor');
    
    % give names and units to data
    data.InputName = 'PWM';
    data.InputUnit = 'duty cycle';
    data.OutputName = {'Angular position', 'Angular velocity'};
    data.OutputUnit = {'rad', 'rad/s'};
    
    % configure starting time and give name to time unit
    data.Tstart = 0;
    data.TimeUnit = 's';
    
    % select guesses for parameters
    p_init = [-0.2 0.7 4]';
    
    % create an initial system with identifiable parameters
    % and the dynamical model given in DC_motor_3_parameters.m
    init_sys = idgrey('DC_motor_3_parameters',p_init,'d');
    
    % conduct model-based parameter estimation (MBPE)
    % (i.e., grey-box model estimation)
    sys = greyest(data,init_sys);
    
    % compare the experiment data with model output
    % using the estimated parameters
    opt = compareOptions('InitialCondition','zero');
    compare(data,sys,Inf,opt)
    
    % record matrices of the identified state space model
    A = sys.A;
    B = sys.B;
    
end

function [A,B,C,D] = DC_motor_3_parameters(p,T_s)
    
    % define discrete-time linear model of a DC motor
    % with three identifiable parameters in state space form
    
    A = [1 T_s;
        p(1) p(2)];
    
    B = [0;p(3)];
    
    C = eye(2);
    
    D = [0;0];
    
end
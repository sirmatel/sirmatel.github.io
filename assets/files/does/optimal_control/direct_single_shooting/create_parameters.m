function d = create_parameters(d)

    % values taken from:
    % "Chen, H., & Allgöwer, F. (1998). 
    % A quasi-infinite horizon nonlinear model predictive control scheme 
    % with guaranteed stability. Automatica, 34(10), 1205-1217."
    % https://doi.org/10.1016/S0005-1098(98)00073-9

    % sampling time [s]
    d.p.T_s = 0.1;

    % OCP horizon length
    d.p.N_c = 15;

    % state dimension
    d.p.n_x = 2;

    % control input dimension
    d.p.n_u = 1;

    % model parameter mu
    d.p.mu = 0.5;

    % true parameter vector
    d.p.p = d.p.mu;

    % create initial state
    d.p.x0_bar = [-0.7;-0.85];
    % warning: choosing x0_bar too large can make the problem infeasible!

    % choose optimization solver
    d.p.optim_solver = 'ipopt';

    % state bounds
    d.p.x_min = -5*ones(d.p.n_x,1);
    d.p.x_max = 5*ones(d.p.n_x,1);

    % control input bounds
    d.p.u_min = -2*ones(d.p.n_u,1);
    d.p.u_max = 2*ones(d.p.n_u,1);

    % objective weighting matrices
    d.p.Q = 0.5*eye(d.p.n_x);
    d.p.R = 1;

    % terminal cost weighting matrix
    d.p.P = [16.5926 11.5926;
        11.5926 16.5926];

    % terminal constraint parameter
    d.p.alpha = 0.7;

end
function d = regulation_NMPC_MPCTools(x1_0,x2_0)
    
    d.c.mpc = import_mpctools();
    
    d.p.x0 = [x1_0 x2_0]';
    
    d = build_setup(d);
    
    d = create_simulator(d);
    
    d = create_NMPC(d);
    
    % simulate
    for t = 1:d.p.t_final
        
        d = solve_NMPC(d,t);
        
        d = evolve_dynamics(d,t);
        
        display(d.s.x(:,t))
        
    end
    
    % delete the field "c"
    % (i.e., MPCTools objects)
    d = rmfield(d,'c');
    
end

function d = build_setup(d)
    
    % state constraints
    d.p.x_min = -Inf;
    d.p.x_max = Inf;
    
    % control input constraints
    d.p.u_min = -2;
    d.p.u_max = 2;
    
    % NMPC prediction horizon (in number of time steps)
    d.p.N_NMPC = 15;
    
    % sampling time (in time units)
    d.p.T = 0.1;
    
    % number of state variables
    d.p.n_x = 2;
    
    % number of control inputs
    d.p.n_u = 1;
    
    % simulation length (in number of time steps)
    d.p.t_final = 120;
    
    % pre-allocate memory
    d.s.x = NaN(d.p.n_x,d.p.t_final);
    d.s.u = NaN(d.p.n_u,d.p.t_final);
    
    % set initial state
    % (d.p.x0 is an argument of the main function)
    d.s.x(:,1) = d.p.x0;
    
    % state constraints vector
    d.p.x_min_v = d.p.x_min*ones(d.p.n_x,1);
    d.p.x_max_v = d.p.x_max*ones(d.p.n_x,1);
    
    % control input constraints vector
    d.p.u_min_v = d.p.u_min*ones(d.p.n_u,1);
    d.p.u_max_v = d.p.u_max*ones(d.p.n_u,1);
    
    % weighting matrices of stage cost
    d.p.Q = 0.5*eye(d.p.n_x);
    d.p.R = eye(d.p.n_u);
    
    % weighting matrix and bound for terminal cost and constraint
    d.p.P = [16.5926 11.5926;11.5926 16.5926];
    d.p.alpha = 0.7;
    
    global Q R P alpha
    Q = d.p.Q;
    R = d.p.R;
    P = d.p.P;
    alpha = d.p.alpha;
    
end

function dxdt = define_dynamics(x, u)
    
    dxdt = [x(2) + u*(0.5 + 0.5*x(1));
        x(1) + u*(0.5 - 2*x(2))];
    
end

function d = create_simulator(d)

    d.c.simulator = ...
        d.c.mpc.getCasadiIntegrator(@define_dynamics, ...
        d.p.T, [d.p.n_x, d.p.n_u], {'x', 'u'});
    
end

function d = create_NMPC(d)
    
    % import dynamics
    ode_casadi_NMPC = d.c.mpc.getCasadiFunc(...
        @define_dynamics, ...
        [d.p.n_x, d.p.n_u], ...
        {'x', 'u'});
    
    % discretize dynamics in time
    F = d.c.mpc.getCasadiFunc(...
        ode_casadi_NMPC, ...
        [d.p.n_x, d.p.n_u], ...
        {'x', 'u'}, ...
        'rk4', true(), ...
        'Delta', d.p.T);
    
    % define stage cost
    l = d.c.mpc.getCasadiFunc(@define_stage_cost, ...
        [d.p.n_x, d.p.n_u], ...
        {'x', 'u'});
    
    % define terminal cost
    Vf = d.c.mpc.getCasadiFunc(@define_terminal_cost, ...
        d.p.n_x, {'x'}, {'Vf'});
    
    % define terminal state constraint
    ef = d.c.mpc.getCasadiFunc(@define_terminal_constraint, ...
        d.p.n_x, {'x'}, {'ef'});
    
    % define NMPC arguments
    commonargs.l = l;
    commonargs.lb.x = d.p.x_min_v;
    commonargs.ub.x = d.p.x_max_v;
    commonargs.lb.u = d.p.u_min_v;
    commonargs.ub.u = d.p.u_max_v;
    commonargs.Vf = Vf;
    commonargs.ef = ef;
    
    % define NMPC problem dimensions
    N.x = d.p.n_x; % state dimension
    N.u = d.p.n_u; % control input dimension
    N.t = d.p.N_NMPC; % time dimension (i.e., prediction horizon)
    
    % create NMPC solver
    d.c.solvers.NMPC = d.c.mpc.nmpc(...
        'f', F, ... % dynamics (discrete-time)
        'N', N, ... % problem dimensions
        'Delta', d.p.T, ... % timestep
        'timelimit', 1, ... % solver time limit (in seconds)
        '**', commonargs); % arguments
    
end

function l = define_stage_cost(x,u)
    
    global Q R
    
    l = x'*Q*x + u'*R*u;
    
end

function Vf = define_terminal_cost(x)
    
    global P
    
    Vf = x'*P*x;
    
end

function ef = define_terminal_constraint(x)
    
    global P alpha
    
    ef = x'*P*x - alpha;
    
end

function d = solve_NMPC(d,t)
    
    % set state at time t as NMPC initial state
    d.c.solvers.NMPC.fixvar('x', 1, d.s.x(:,t));
    
    tic_c = tic;
    
    % solve NMPC problem
    d.c.solvers.NMPC.solve();
    
    % record CPU time
    d.s.CPU_time(t,1) = toc(tic_c);
    
    % assign first element of the solution to the NMPC
    % problem as the control input at time t
    d.s.u(:,t) = d.c.solvers.NMPC.var.u(:,1);
    
    % record the predicted state trajectory
    % for the first time step
    if t == 1
        
        d.p.x_NMPC_t_1 = d.c.solvers.NMPC.var.x;
        
    end
    
end

function d = evolve_dynamics(d,t)
    
    d.s.x(:,t+1) = ...
        full(d.c.simulator(d.s.x(:,t), d.s.u(:,t)));
    
end

function d = NMHE_MPCTools(x1_0,x2_0)
    
    dbstop if error
    
    d.c.mpc = import_mpctools();
    
    d.p.x0 = [x1_0 x2_0]';
    
    d = build_setup(d);
    
    d = create_simulator(d);
    
    d = create_NMHE(d);
    
    % simulate
    for t = 1:d.p.t_final
        
        % take measurement
        d.s.y(:,t) = full(d.c.h(d.s.x(:,t))) + d.s.v(:,t);
        
        d = solve_NMHE(d,t);
        
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
    
    % NMHE estimation horizon (in number of time steps)
    d.p.N_NMHE = 15;
    
    % sampling time (in time units)
    d.p.T = 0.1;
    
    % number of state variables
    d.p.n_x = 2;
    
    % number of process noise variables
    d.p.n_w = 2;
    
    % number of measurements
    d.p.n_y = 2;
    
    % number of measurement noise variables
    d.p.n_v = 2;
    
    % simulation length (in number of time steps)
    d.p.t_final = 120;
    
    % pre-allocate memory
    d.s.x = NaN(d.p.n_x,d.p.t_final); % state
    d.s.x_hat = NaN(d.p.n_x,d.p.t_final); % state estimate
    d.s.y = NaN(d.p.n_y,d.p.t_final); % measurement
    d.s.CPU_time = NaN(d.p.t_final,1); % NMHE CPU time
    
    % set initial state
    d.s.x(:,1) = d.p.x0;
    
    % process noise standard deviations
    d.p.sigma_w = [0.1 0.1]';
    
    % measurement noise standard deviations
    d.p.sigma_v = [1 1]';
    
    % process noise covariance
    d.p.Sigma_w = diag(d.p.sigma_w.^2);
    
    % process noise inverse covariance
    d.p.Q = d.c.mpc.spdinv(d.p.Sigma_w);
    
    % measurement noise covariance
    d.p.Sigma_v = diag(d.p.sigma_v.^2);
    
    % measurement noise inverse covariance
    d.p.R = d.c.mpc.spdinv(d.p.Sigma_v);
    
    % set random number generator seed to 0
    rng(0)
    
    % create process noise signal
    d.s.w = diag(d.p.sigma_w)*randn(d.p.n_w, d.p.t_final);
    
    % create measurement noise signal
    d.s.v = diag(d.p.sigma_v)*randn(d.p.n_v, d.p.t_final);
    
    % state constraints vector
    d.p.x_min_v = d.p.x_min*ones(d.p.n_x,1);
    d.p.x_max_v = d.p.x_max*ones(d.p.n_x,1);
    
end

function dxdt = define_dynamics(x, w)
    
    dxdt = [x(2) + w(1);
        (1-(x(1)^2))*x(2) - x(1) + w(2)];
    
end

function d = create_simulator(d)
    
    d.c.simulator = ...
        d.c.mpc.getCasadiIntegrator(@define_dynamics, ...
        d.p.T, [d.p.n_x, d.p.n_w], {'x', 'w'});
    
end

function d = create_NMHE(d)
    
    % import dynamics
    ode_casadi_NMHE = d.c.mpc.getCasadiFunc(...
        @define_dynamics, ...
        [d.p.n_x, d.p.n_w], ...
        {'x', 'w'});
    
    % discretize dynamics in time
    F = d.c.mpc.getCasadiFunc(...
        ode_casadi_NMHE, ...
        [d.p.n_x, d.p.n_w], ...
        {'x', 'w'}, ...
        'rk4', true(), ...
        'Delta', d.p.T);
    
    % define stage cost
    l = d.c.mpc.getCasadiFunc(@(w, v) ...
        w'*d.p.Q*w + v'*d.p.R*v, ...
        [d.p.n_w, d.p.n_v], ...
        {'w', 'v'});
    
    % define measurement equation
    d.c.h = d.c.mpc.getCasadiFunc(@(x) x, ...
        d.p.n_x, {'x'}, {'H'});
    
    % define NMHE arguments
    commonargs.l = l;
    commonargs.h = d.c.h;
    commonargs.y = zeros(d.p.n_y,d.p.N_NMHE+1);
    commonargs.lb.x = repmat(d.p.x_min_v,1,d.p.N_NMHE+1);
    commonargs.ub.x = repmat(d.p.x_max_v,1,d.p.N_NMHE+1);
    
    % define NMHE problem dimensions
    N.x = d.p.n_x; % state dimension
    N.w = d.p.n_w; % process noise dimension
    N.y = d.p.n_y; % measurement dimension
    N.t = d.p.N_NMHE; % time dimension (i.e., estimation horizon)
    
    % create NMHE solver
    d.c.solvers.NMHE = d.c.mpc.nmhe(...
        'f', F, ...
        'N', N, ...
        '**', commonargs, ...
        'priorupdate', 'none');
    
end

function d = solve_NMHE(d,t)
    
    % record new measurement
    d.c.solvers.NMHE.newmeasurement(d.s.y(:,t));
    
    tic_e = tic;
    
    % solve NMHE problem
    d.c.solvers.NMHE.solve();
    
    % record CPU time
    d.s.CPU_time(t,1) = toc(tic_e);
    
    % assign last element of the state trajectory (found
    % as NMHE solution) as the state estimate at time t
    d.s.x_hat(:,t) = d.c.solvers.NMHE.var.x(:,end);
    
end

function d = evolve_dynamics(d,t)
    
    if t < d.p.t_final
        
        d.s.x(:,t+1) = ...
            full(d.c.simulator(d.s.x(:,t), d.s.w(:,t)));
        
    end
    
end

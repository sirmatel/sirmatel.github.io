function d = MBPE_meas_noise_CasADi()
    
    d = build_setup();
    
    d = define_dynamics(d);
    
    d = create_simulator(d);
    
    d = simulate_system(d);
    
    d = solve_MBPE_original(d,d.p.p_guess);
    
    % record solution using the original MBPE formulation
    d.p.p_hat_MBPE_original_default = d.p.p_hat_MBPE_original;
    
    d = solve_MBPE_convex(d,d.p.p_guess);
    
    d = solve_MBPE_original(d,d.p.p_hat_MBPE_convex);
    
    % record solution using the original MBPE formulation
    % with the initial guess set to solution of the convex approximation
    d.p.p_hat_MBPE_original_convex_initialized = d.p.p_hat_MBPE_original;
    
    d.c = [];
    
end

function d = build_setup()
    
    % sampling time (in time units)
    d.p.T_s = 0.1;
    
    % experiment horizon (in time units)
    d.p.T_exp = 10;
    
    % experiment horizon (in time steps)
    d.p.k_exp = ceil(d.p.T_exp/d.p.T_s);
    
    % true parameter
    d.p.p_true = 10;
    
    % parameter scaling
    d.p.p_scale = 1;
    
    % parameter guess
    d.p.p_guess = 4;
    
    % state dimension
    d.p.n_x = 2;
    
    % parameter dimension
    d.p.n_p = 1;
    
    % measurement noise dimension
    d.p.n_v = 2;
    
    % memory preallocation for state trajectory
    d.s.x = NaN(d.p.n_x,d.p.k_exp);
    
    % initial state
    d.p.x0 = [0;1];
    d.s.x(:,1) = d.p.x0;
    
    % measurement noise standard deviations
    d.p.sigma_v = [0.1 0.1]';
    
    % set random number generator seed to 0
    % for repeatability
    rng(0)
    
    % measurement noise signal
    d.s.v = diag(d.p.sigma_v)*randn(d.p.n_v, d.p.k_exp);
    
    % matrix penalizing mismatch between state and measurement
    d.p.Q = eye(d.p.n_x);
    
end

function d = define_dynamics(d)
    
    import casadi.*
    
    % state variables
    x1  = MX.sym('x1');
    x2  = MX.sym('x2');
    
    % state variables vector
    states = [x1;x2];
    
    % parameters
    p  = MX.sym('p');
    
    % parameters vector
    params = p;
    
    % dynamics
    x_dot = [x2;-x1*p];
    
    % dynamics as a CasAdi function
    d.c.f = Function('f',{states,params},{x_dot});
    
    % record states and variables CasAdi objects
    d.c.states = states;
    d.c.params = params;
    
end

function d = create_simulator(d)
    
    import casadi.*
    
    % create RK4 integrator
    k1 = d.c.f(d.c.states,d.c.params);
    k2 = d.c.f(d.c.states + (d.p.T_s/2)*k1,d.c.params);
    k3 = d.c.f(d.c.states + (d.p.T_s/2)*k2,d.c.params);
    k4 = d.c.f(d.c.states + d.p.T_s*k3,d.c.params);
    
    states_final = (d.p.T_s/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    % create a function conducts one-step ahead simulation
    % from the current state
    d.c.Fflow = Function('Fflow',{d.c.states, d.c.params},{states_final});
    
end

function d = simulate_system(d)
    
    % simulate system with the true parameter
    for k = 1:d.p.k_exp-1
        
        d.s.x(:,k+1) = d.s.x(:,k) + ...
            full(d.c.Fflow(d.s.x(:,k),d.p.p_true));
        
    end
    
    % create measurement trajectory by adding
    % measurement noise to the state trajectory
    d.s.y = d.s.x + d.s.v;
    
end

function d = solve_MBPE_original(d,p_guess)
    
    import casadi.*
    
    opti = casadi.Opti();
    
    % measurement trajectory
    y = d.s.y;
    
    % define state trajectory as decision variable
    x = opti.variable(d.p.n_x,d.p.k_exp);
    
    % define parameters as decision variable
    p = opti.variable(d.p.n_p,1);
    
    % initialize objective function
    J = 0;
    
    % construct state trajectory by simulating the system
    % as a function of parameters
    for k = 1:d.p.k_exp-1
        
        % simulate one step forward
        x_plus = x(:,k) + d.c.Fflow(x(:,k),p.*d.p.p_scale);
        
        % ensure continuity of state trajectory
        % (i.e., direct multiple shooting)
        opti.subject_to(x(:,k+1) == x_plus)
        
    end
    
    for k = 1:d.p.k_exp
        
        % add penalization of mismatch between
        % state and measurement to the objective function
        J = J + (x(:,k) - y(:,k))'*d.p.Q*(x(:,k) - y(:,k));
        
    end
    
    % configure solver options
    opti.solver('ipopt')
    p_opts = struct('expand',true);
    s_opts = struct('max_iter',100);
    opti.solver('ipopt',p_opts,s_opts);
    
    % set initial guess for parameters
    opti.set_initial(p,p_guess./d.p.p_scale);
    
    % set J as the objective function
    opti.minimize(J)
    
    try
        
        % attempt to solve the optimization problem
        sol = opti.solve();
        
        % undo scaling and record solution as the
        % parameter estimate
        d.p.p_hat_MBPE_original = sol.value(p).*d.p.p_scale;
        
    catch
        
        % give warning if the problem was not
        % successfully solved
        warning('problem during optimization!')
        
    end
    
end

function d = solve_MBPE_convex(d,p_guess)
    
    import casadi.*
    
    opti = casadi.Opti();
    
    % measurement trajectory
    y = d.s.y;
    
    % define pseudo state trajectory as decision variable
    x_c = opti.variable(d.p.n_x,d.p.k_exp);
    
    % define parameters as decision variable
    p = opti.variable(d.p.n_p,1);
    
    % initialize objective function
    J = 0;
    
    % construct state trajectory by simulating the system
    % as a function of parameters
    for k = 1:d.p.k_exp-1
        
        % simulate one step forward
        x_c_plus = x_c(:,k) + d.c.Fflow(y(:,k),p.*d.p.p_scale);
        
        % ensure continuity of state trajectory
        % (i.e., direct multiple shooting)
        opti.subject_to(x_c(:,k+1) == x_c_plus)
        
    end
    
    for k = 1:d.p.k_exp
        
        % add penalization of mismatch between
        % pseudo state and measurement to the objective function
        J = J + (x_c(:,k) - y(:,k))'*d.p.Q*(x_c(:,k) - y(:,k));
        
    end
    
    % configure solver options
    opti.solver('ipopt')
    p_opts = struct('expand',true);
    s_opts = struct('max_iter',100);
    opti.solver('ipopt',p_opts,s_opts);
    
    % set initial guess for parameters
    opti.set_initial(p,p_guess./d.p.p_scale);
    
    % set J as the objective function
    opti.minimize(J)
    
    try
        
        % attempt to solve the optimization problem
        sol = opti.solve();
        
        % undo scaling and record solution as the
        % parameter estimate
        d.p.p_hat_MBPE_convex = sol.value(p).*d.p.p_scale;
        
    catch
        
        % give warning if the problem was not
        % successfully solved
        warning('problem during optimization!')
        
    end
    
end
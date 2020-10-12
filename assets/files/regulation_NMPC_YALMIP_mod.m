function d = regulation_NMPC_YALMIP_mod(x1_0,x2_0)
    
    % same as regulation_NMPC_YALMIP, however,
    % uses "optimize" calls instead of "optimizer" object
    
    d.p.x0 = [x1_0;x2_0];
    
    d = build_setup(d);
    
    for t = 1:d.p.t_final
        
        d = solve_NMPC(d,t);
        
        d = evolve_dynamics(d,t);
        
        display(d.s.x(:,t))
        
    end
    
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
    
end

function dxdt = define_dynamics(t,x,u)
    
    mu = 0.5;
    
    dxdt = [x(2) + u*(mu + (1-mu)*x(1)); ...
        x(1) + u*(mu - 4*(1-mu)*x(2))];
    
end

function x_plus = F(f,x,u,h)
    
    % create RK4 integrator
    k1 = f(0,x,u);
    k2 = f(0,x+(h/2).*k1,u);
    k3 = f(0,x+(h/2).*k2,u);
    k4 = f(0,x+h.*k3,u);
    x_plus = x + (h/6).*(k1+2*k2+2*k3+k4);
    
end

function d = solve_NMPC(d,t)
    
    tic_c = tic;
    
    % define control inputs as decision variables
    u = sdpvar(d.p.n_u,d.p.N_NMPC,'full');
    
    % define states as decision variables
    x = sdpvar(d.p.n_x,d.p.N_NMPC+1,'full');
    
    % initialize objective function
    obj = 0;
    
    % initialize constraints
    con = [];
    
    % initial state constraint
    con = [con, x(:,1) == d.s.x(:,t)];
    
    for k = 1:d.p.N_NMPC
        
        % ensure continuity of state trajectory
        % (i.e., direct multiple shooting)
        con = [con, x(:,k+1) == F(@define_dynamics, x(:,k), u(:,k), d.p.T)];
        
    end
    
    for k = 1:d.p.N_NMPC
        
        % add stage cost to objective function
        obj = obj + x(:,k)'*d.p.Q*x(:,k) + u(:,k)'*d.p.R*u(:,k);
        
    end
    
    for k = 1:d.p.N_NMPC
        
        % control input constraints
        con = [con, d.p.u_min_v <= u(:,k) <= d.p.u_max_v];
        
        % state constraints
        con = [con, d.p.x_min_v <= x(:,k) <= d.p.x_max_v];
        
    end
    
    % add terminal cost to objective function
    obj = obj + x(:,d.p.N_NMPC+1)'*d.p.P*x(:,d.p.N_NMPC+1);
    
    % terminal constraint
    con = [con, x(:,d.p.N_NMPC+1)'*d.p.P*x(:,d.p.N_NMPC+1) <= d.p.alpha];
    
    % define optimization settings
    ops = sdpsettings;
    ops.verbose = 0; % configure level of info solver displays
    ops.solver = 'ipopt'; % choose solver
    
    % solve NMPC problem and record diagnostics
    d.s.diagnostics{t} = optimize(con,obj,ops);
    
    % record primal and dual constraint violations
    [primalfeas,dualfeas] = check(con);
    d.s.primalfeas{t} = primalfeas;
    d.s.dualfeas{t} = dualfeas;
    
    % record CPU time
    d.s.CPU_time(t,1) = toc(tic_c);
    
    % extract the control input trajectory
    % from the solution of MPC optimization problem
    u_t = value(u);
    
    % record the predicted state trajectory
    % for the first time step
    if t == 1
        
        % extract the state trajectory
        % from the solution of MPC optimization problem
        x_t = value(x);
        
        d.p.x_NMPC_t_1 = x_t;
        
    end
    
    % assign first element of the solution to the NMPC
    % problem as the control input at time t
    d.s.u(:,t) = u_t(:,1);
    
end

function d = evolve_dynamics(d,t)
    
    d.s.x(:,t+1) = ...
        F(@define_dynamics, d.s.x(:,t), d.s.u(:,t), d.p.T);
    
end
function MPC_controller = MBPE_MPC_DC_motor_create_MPC(A,B,Q,R,N)
    
    % define control inputs as decision variables
    u = sdpvar(1,N,'full');
    
    % define states as decision variables
    x = sdpvar(2,N+1,'full');
    
    % define angular position reference as decision variables
    r = sdpvar(1,N+1,'full');
    
    % initialize objective function
    obj = 0;
    
    % initialize constraints
    con = [];
    
    for k = 1:N
        
        % ensure continuity of state trajectory
        % (i.e., direct multiple shooting)
        con = [con, x(:,k+1) == A*x(:,k) + B*u(:,k)];
        
    end
    
    for k = 1:N+1
        
        % add stage cost for states to objective function
        obj = obj + (x(1,k) - r(1,k))'*Q*(x(1,k) - r(1,k));
        
    end
    
    for k = 1:N
        
        % add stage cost for inputs to objective function
        obj = obj + u(:,k)'*R*u(:,k);
        
    end
    
    for k = 1:N
        
        % control input constraints
        con = [con, -1 <= u(:,k) <= 1];
        
        con = [con, -4 <= x(2,k+1) <= 4];
        
    end
    
    % define optimization settings
    ops = sdpsettings;
    ops.verbose = 0; % configure level of info solver displays
    ops.solver = 'ipopt'; % choose solver
    ops.ipopt.max_cpu_time = 0.01;
     
    % configure the inputs of the controller object
    % (we choose the predicted control input and state trajectories)
    controller_inputs = [x(:,1);r(:)];
    
    % configure the outputs of the controller object
    % (we choose the predicted control input and state trajectories)
    controller_outputs = [u(:);x(:)];
    
    % create MPC controller object
    MPC_controller = optimizer(con,obj,ops,...
        controller_inputs,...
        controller_outputs);
    
end
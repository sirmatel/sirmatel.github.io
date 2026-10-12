function d = create_trajectories(d)
    
    % state dimension
    d.p.n_x = size(d.c.x,1);

    % control input dimension
    d.p.n_u = size(d.c.u,1);

    % process noise dimension
    d.p.n_w = size(d.c.w,1);

    % measurement noise dimension
    d.p.n_v = size(d.c.v,1);

    % create state trajectory (memory preallocation)
    d.s.x = NaN(d.p.n_x,d.p.kappa_max+1);
    
    % create initial state
    d.p.x0 = zeros(d.p.n_x,1);

    % initialize state trajectory from initial state
    d.s.x(:,1) = d.p.x0;

    % create control input trajectory (memory preallocation)
    d.s.u = NaN(d.p.n_u,d.p.kappa_max);

    % create process noise trajectory
    d.s.w = zeros(d.p.n_w,d.p.kappa_max);

    % create measurement noise trajectory
    d.s.v = zeros(d.p.n_v,d.p.kappa_max);

end
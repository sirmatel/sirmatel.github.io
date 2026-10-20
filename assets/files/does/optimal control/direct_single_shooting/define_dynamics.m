function d = define_dynamics(d)

    import casadi.*

    % states
    x1 = MX.sym('x1');
    x2 = MX.sym('x2');
    
    % state vector
    x = [x1; x2];

    % control inputs
    u = MX.sym('u');
    
    % parameters
    mu = MX.sym('mu');
    
    p = mu;

    % dynamics equations

    x1_dot = x2 + u*(mu + (1 - mu)*x1);
    
    x2_dot = x1 + u*(mu - 4*(1 - mu)*x2);

    x_dot = [x1_dot; x2_dot];

    l = (x'*d.p.Q*x) + (u'*d.p.R*u);

    % dynamics function (continuous time)
    d.c.f = Function('f',{x, u, p},{x_dot, l});

    d.c.x = x;
    d.c.u = u;
    d.c.p = p;
    
end
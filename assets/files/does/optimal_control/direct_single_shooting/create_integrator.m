function d = create_integrator(d)

    import casadi.*

    [k1_F,k1_L] = d.c.f(d.c.x, d.c.u, d.c.p);
    [k2_F,k2_L] = d.c.f(d.c.x + (d.p.T_s/2)*k1_F, d.c.u, d.c.p);
    [k3_F,k3_L] = d.c.f(d.c.x + (d.p.T_s/2)*k2_F, d.c.u, d.c.p);
    [k4_F,k4_L] = d.c.f(d.c.x + d.p.T_s*k3_F, d.c.u, d.c.p);

    F_expression = d.c.x + (d.p.T_s/6)*(k1_F + 2*k2_F + 2*k3_F + k4_F);

    L_expression = (d.p.T_s/6)*(k1_L + 2*k2_L + 2*k3_L + k4_L);

    G_expression = ...
        [d.p.x_min - d.c.x; % x_min <= x
        d.c.x - d.p.x_max; % x <= x_max
        d.p.u_min - d.c.u; % u_min <= u
        d.c.u - d.p.u_max]; % u <= u_max

    M_expression = d.c.x'*d.p.P*d.c.x;

    R_expression = d.c.x'*d.p.P*d.c.x - d.p.alpha; % x'*P*x <= alpha

    % dynamics function (discrete time)
    d.c.F = Function('F',{d.c.x, d.c.u, d.c.p}, {F_expression});

    % stage cost function (discrete time)
    d.c.L = Function('L',{d.c.x, d.c.u, d.c.p}, {L_expression});

    % path constraint function
    d.c.G = Function('G',{d.c.x, d.c.u}, {G_expression});

    % terminal cost function
    d.c.M = Function('M',{d.c.x},{M_expression});

    % terminal constraint function
    d.c.R = Function('R',{d.c.x}, {R_expression});

end
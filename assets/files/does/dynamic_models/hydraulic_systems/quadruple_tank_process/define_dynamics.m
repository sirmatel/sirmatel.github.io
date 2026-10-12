function d = define_dynamics(d)

    import casadi.*

    % states: water levels [cm]
    h1 = MX.sym('h1');
    h2 = MX.sym('h2');
    h3 = MX.sym('h3');
    h4 = MX.sym('h4');

    % state vector
    x = [h1; h2; h3; h4];

    % control inputs: pump voltages [V]
    u_v1 = MX.sym('uv1');
    u_v2 = MX.sym('uv2');

    % control input vector
    u = [u_v1; u_v2];

    % process noise terms
    w1 = MX.sym('w1');
    w2 = MX.sym('w2');
    w3 = MX.sym('w3');
    w4 = MX.sym('w4');

    % process noise vector
    w = [w1; w2; w3; w4];

    % measurement noise terms
    v1 = MX.sym('v1');
    v2 = MX.sym('v2');
    v3 = MX.sym('v3');
    v4 = MX.sym('v4');

    % measurement noise vector
    v = [v1; v2; v3; v4];

    % parameters
    A1 = MX.sym('A1');  % tank cross-sections [cm^2]
    A2 = MX.sym('A2');
    A3 = MX.sym('A3');
    A4 = MX.sym('A4');
    a1 = MX.sym('a1');  % outlet cross-sections [cm^2]
    a2 = MX.sym('a2');
    a3 = MX.sym('a3');
    a4 = MX.sym('a4');
    gamma1 = MX.sym('gamma1');  % valve flow-split ratios (unitless)
    gamma2 = MX.sym('gamma2');
    k1 = MX.sym('k1');  % pump gains [cm^3/(V s)]
    k2 = MX.sym('k2');
    g = MX.sym('g'); % gravitational acceleration [cm/s^2]

    p = [A1; A2; A3; A4; a1; a2; a3; a4; gamma1; gamma2; k1; k2; g];

    % dynamics equations

    h1_dot = (-a1/A1)*sqrt(2*g*h1) + ...
        (a3/A1)*sqrt(2*g*h3) + gamma1*(k1/A1)*u_v1 + w1;

    h2_dot = (-a2/A2)*sqrt(2*g*h2) + ...
        (a4/A2)*sqrt(2*g*h4) + gamma2*(k2/A2)*u_v2 + w2;

    h3_dot = (-a3/A3)*sqrt(2*g*h3) + (1-gamma2)*(k2/A3)*u_v2 + w3;

    h4_dot = (-a4/A4)*sqrt(2*g*h4) + (1-gamma1)*(k1/A4)*u_v1 + w4;

    x_dot = [h1_dot; h2_dot; h3_dot; h4_dot];

    % measurement equations
    y1 = x(1) + v1;
    y2 = x(2) + v2;
    y3 = x(3) + v3;
    y4 = x(4) + v4;

    y = [y1; y2; y3; y4];

    % dynamics function
    d.c.f = Function('f',{x, u, p, w, v},{x_dot});

    % measurement function
    d.c.H = Function('H',{x, u, p, w, v},{y});

    d.c.x = x;
    d.c.u = u;
    d.c.w = w;
    d.c.v = v;
    d.c.p = p;
    
end
function d = create_parameters(d)

    % sampling time [s]
    d.p.T_s = 10;

    % simulation length
    % (in number of discrete time steps)
    d.p.kappa_max = 40;

    % fixed control input
    d.p.u_fixed = [3;3];

    % tank cross-sections [cm^2]
    A1 = 28;
    A2 = 32;
    A3 = 28;
    A4 = 32;

    % outlet cross-sections [cm^2]
    a1 = 0.071;
    a2 = 0.057;
    a3 = 0.071;
    a4 = 0.057;

    % valve flow-split ratios (unitless)
    gamma1 = 0.7;
    gamma2 = 0.6;

    % pump gains [cm^3/(V s)]
    k1 = 3.33;
    k2 = 3.35;

    % gravitational acceleration [cm/s^2]
    g = 981;

    % true parameter vector
    d.p.p = [A1; A2; A3; A4; a1; a2; a3; a4; gamma1; gamma2; k1; k2; g];

end
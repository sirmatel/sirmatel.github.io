function d = solve_OCP_direct_multiple_shooting(d)

    import casadi.*

    opti = casadi.Opti();

    x = opti.variable(d.p.n_x,d.p.N_c+1);
    u = opti.variable(d.p.n_u,d.p.N_c);

    opti.subject_to(x(:,1) == d.p.x0_bar)

    J = 0;

    for k = 1:d.p.N_c

        J = J + d.c.L(x(:,k),u(:,k),d.p.p);

        opti.subject_to(d.c.G(x(:,k),u(:,k)) <= 0)

        x_next = d.c.F(x(:,k),u(:,k),d.p.p);

        opti.subject_to(x(:,k+1) == x_next)

    end

    J = J + d.c.M(x(:,d.p.N_c+1));

    opti.subject_to(d.c.R(x(:,d.p.N_c+1)) <= 0)

    opti.solver(d.p.optim_solver)

    opti.minimize(J)

    sol = opti.solve();

    d.s.x = sol.value(x);
    d.s.u = sol.value(u);

end
function d = solve_OCP_direct_single_shooting(d)

    import casadi.*

    opti = casadi.Opti();

    u = opti.variable(d.p.n_u,d.p.N_c);

    x = d.p.x0_bar;

    J = 0;

    for k = 1:d.p.N_c

        J = J + d.c.L(x,u(:,k),d.p.p);

        opti.subject_to(d.c.G(x,u(:,k)) <= 0)

        x = d.c.F(x,u(:,k),d.p.p);

    end

    J = J + d.c.M(x);

    opti.subject_to(d.c.R(x) <= 0)

    opti.solver(d.p.optim_solver)

    opti.minimize(J)

    sol = opti.solve();

    d.s.u = sol.value(u);

end
function d = create_integrator(d)

    import casadi.*

    k1 = d.c.f(d.c.x, d.c.u, d.c.p, d.c.w, d.c.v);
    k2 = d.c.f(d.c.x + (d.p.T_s/2)*k1, d.c.u, d.c.p, d.c.w, d.c.v);
    k3 = d.c.f(d.c.x + (d.p.T_s/2)*k2, d.c.u, d.c.p, d.c.w, d.c.v);
    k4 = d.c.f(d.c.x + d.p.T_s*k3, d.c.u, d.c.p, d.c.w, d.c.v);

    F_expression = d.c.x + (d.p.T_s/6)*(k1 + 2*k2 + 2*k3 + k4);

    d.c.F = Function('F',{d.c.x, d.c.u, d.c.p, d.c.w, d.c.v}, {F_expression});

end
function d = simulate_system(d)
   
    d.s.x = NaN(d.p.n_x,d.p.N_c+1);
    d.s.x(:,1) = d.p.x0_bar;

    % simulation for loop
    for kappa = 1:d.p.N_c

        d.s.x(:,kappa+1) = ...
            full(d.c.F(d.s.x(:,kappa), ...
            d.s.u(:,kappa), ...
            d.p.p));

    end

end
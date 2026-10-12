function d = run_simulation()
    
    d = [];

    d = create_parameters(d);

    d = define_dynamics(d);

    d = create_integrator(d);

    d = create_trajectories(d);

    % simulation for loop
    for kappa = 1:d.p.kappa_max

        d.s.u(:,kappa) = d.p.u_fixed;

        d.s.x(:,kappa+1) = ...
            full(d.c.F(d.s.x(:,kappa), ...
            d.s.u(:,kappa), ...
            d.p.p, ...
            d.s.w(:,kappa), ...
            d.s.v(:,kappa)));

    end

    % delete CasADi objects
    % (to avoid problems with saving/loading mat files)
    d = rmfield(d,'c');

    % plot state trajectories
    m_sp = ceil(sqrt(d.p.n_x)); % number of rows (subplot grid)
    n_sp = ceil(sqrt(d.p.n_x)); % number of columns (subplot grid)
    for ix = 1:d.p.n_x

        subplot(m_sp,n_sp,ix)
        plot(d.s.x(ix,:))
        xlim([1 d.p.kappa_max])
        xlabel_str = sprintf('$\\kappa$');
        xlabel(xlabel_str, 'Interpreter', 'latex')
        title_str = sprintf('$x_{%g}(\\kappa)$', ix);
        title(title_str, 'Interpreter', 'latex')

    end

end
function d = main()

    d = [];

    d = create_parameters(d);

    d = define_dynamics(d);

    d = create_integrator(d);

    d = solve_OCP_direct_multiple_shooting(d);

    % delete CasADi objects
    % (to avoid problems with saving/loading mat files)
    d = rmfield(d,'c');

    % plot results
    plot_results(d)

end

function plot_results(d)

    lw = 2; % line width
    colorOrder = get(gca, 'ColorOrder');

    subplot(2,2,1)
    plot(d.s.x(1,:),'Color',colorOrder(1,:),'LineWidth',lw)
    xlim([1 d.p.N_c+1])
    xlabel_str = sprintf('$\\kappa$');
    xlabel(xlabel_str, 'Interpreter', 'latex')
    title_str = sprintf('$x_{%g}(\\kappa)$', 1);
    title(title_str, 'Interpreter', 'latex')

    subplot(2,2,2)
    plot(d.s.x(2,:),'Color',colorOrder(1,:),'LineWidth',lw)
    xlim([1 d.p.N_c+1])
    xlabel_str = sprintf('$\\kappa$');
    xlabel(xlabel_str, 'Interpreter', 'latex')
    title_str = sprintf('$x_{%g}(\\kappa)$', 2);
    title(title_str, 'Interpreter', 'latex')

    subplot(2,2,3)
    plot(d.s.u(1,:),'Color',colorOrder(1,:),'LineWidth',lw)
    xlim([1 d.p.N_c])
    xlabel_str = sprintf('$\\kappa$');
    xlabel(xlabel_str, 'Interpreter', 'latex')
    title_str = sprintf('$u(\\kappa)$');
    title(title_str, 'Interpreter', 'latex')

    subplot(2,2,4)
    hold on
    box on
    plot_ellipse(d.p.P,d.p.alpha,colorOrder(4,:),lw)
    plot(d.s.x(1,:),d.s.x(2,:),'Color',colorOrder(1,:),'LineWidth',lw)
    xlabel_str = sprintf('$x_{%g}(\\kappa)$', 1);
    xlabel(xlabel_str, 'Interpreter', 'latex')
    ylabel_str = sprintf('$x_{%g}(\\kappa)$', 2);
    ylabel(ylabel_str, 'Interpreter', 'latex')
    title_str = sprintf('$x_1(\\kappa)-x_2(\\kappa)$');
    title(title_str, 'Interpreter', 'latex')

end

function plot_ellipse(P,alpha,color,line_width)

    theta = linspace(0, 2*pi, 100);
    unit_circle = [cos(theta); sin(theta)];

    L = chol(P, 'lower');
    ellipse_pts = sqrt(alpha)*(L'\unit_circle);

    plot(ellipse_pts(1,:), ellipse_pts(2,:), ...
        'Color', color, 'LineWidth', line_width);

end
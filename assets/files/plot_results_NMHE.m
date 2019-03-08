function plot_results_NMHE(d)
    
    hFig = figure;
    
    fig_left = 50;
    fig_bottom = 50;
    
    fig_height = 900;
    fig_width = 1400;
    
    set(hFig, 'Position', ...
        [fig_left fig_bottom fig_width fig_height])
    
    lw = 3;
    
    subplot(2,2,1)
    hold on
    box on
    stairs(d.s.x(1,:),'LineWidth',3)
    stairs(d.s.y(1,:),'LineWidth',1)
    stairs(d.s.x_hat(1,:),'LineWidth',2)
    xlabel('$t$')
    title('$x_1(t)$')
    xlim([1 d.p.t_final])
    ylim([-6 6])
    
    subplot(2,2,2)
    hold on
    box on
    stairs(d.s.x(2,:),'LineWidth',3)
    stairs(d.s.y(2,:),'LineWidth',1)
    stairs(d.s.x_hat(2,:),'LineWidth',2)
    xlabel('$t$')
    title('$x_2(t)$')
    legend('true','measured','estimate','location','NorthWest')
    xlim([1 d.p.t_final])
    ylim([-6 6])
    
    subplot(2,2,3)
    hold on
    box on
    plot(d.s.x(1,1:d.p.t_final)-d.s.x_hat(1,1:d.p.t_final),...
        d.s.x(2,1:d.p.t_final)-d.s.x_hat(2,1:d.p.t_final),...
        '-k','LineWidth',lw);
    xlabel('$x_1(t) - \hat{x}_1(t)$')
    ylabel('$x_2(t) - \hat{x}_2(t)$')
    xlim([-2 2])
    ylim([-2 2])
    
    subplot(2,2,4)
    hold on
    box on
    plot(d.s.x(1,:),d.s.x(2,:),'LineWidth',3);
    plot(d.s.x_hat(1,:),d.s.x_hat(2,:),...
        'color',[0.929 0.694 0.125],'LineWidth',2);
    xlabel('$x_1(t)$')
    ylabel('$x_2(t)$')
    legend('true','estimate','location','SouthEast')
    xlim([-3 3])
    ylim([-3 3])
    
end
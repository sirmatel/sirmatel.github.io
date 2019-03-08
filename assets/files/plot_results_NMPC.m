function plot_results_NMPC(d)
    
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
    stairs(d.s.x(1,:),'LineWidth',lw)
    xlabel('$t$')
    title('$x_1(t)$')
    xlim([1 d.p.t_final])
    ylim([-1 1])
    
    subplot(2,2,2)
    hold on
    box on
    stairs(d.s.x(2,:),'LineWidth',lw)
    xlabel('$t$')
    title('$x_2(t)$')
    xlim([1 d.p.t_final])
    ylim([-1 1])
    
    subplot(2,2,3)
    hold on
    box on
    stairs(d.s.u(1,:),'-k','LineWidth',lw)
    xlabel('$t$')
    title('$u(t)$')
    xlim([1 d.p.t_final])
    ylim([-2.5 2.5])
    
    subplot(2,2,4)
    hold on
    box on
    plot(-1:1,zeros(size(-1:1)),'--k','LineWidth',1)
    plot(zeros(size(-1:1)),-1:1,'--k','LineWidth',1)
    h1 = plot_ellipse([16.5926 11.5926;11.5926 16.5926],[0 0]',0.7);
    h2 = plot(d.s.x(1,:),d.s.x(2,:),'Color',[0 0.447 0.741],'LineWidth',lw);
    h3 = plot(d.p.x_NMPC_t_1(1,:),d.p.x_NMPC_t_1(2,:),...
        'color',[0.466 0.674 0.188],'LineWidth',2);
    xlabel('$x_1(t)$')
    ylabel('$x_2(t)$')
    legend([h1 h2 h3],{'terminal region',...
        'state trajectory (simulation)',...
        'state trajectory (predicted at $t = 1$)'},...
        'Location','SouthEast')
    xlim([-1 1])
    ylim([-1 1])
    
end

function h = plot_ellipse(A,c,alpha)
    
    t = linspace(0, 2*pi, 100);
    z = [cos(t); sin(t)];
    r = chol(A/alpha)\z;
    h = plot(c(1)+r(1,:),c(2)+r(2,:),...
        'color',[0.494 0.184 0.556],'LineWidth',3);
    
end
function plot_r(d)
    
    t = 0:d.p.T:((d.p.k_final-1)*d.p.T);
    
    subplot(3,3,1)
    plot(t,d.s.x(1,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('position (m)')
    title('$x$-position (state 1)')
    
    subplot(3,3,2)
    plot(t,d.s.x(2,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('position (m)')
    title('$y$ position (state 2)')
    
    subplot(3,3,3)
    plot(t,d.s.x(3,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('angle (rad)')
    title('$\psi$ (yaw angle) (state 3)')
    
    subplot(3,3,4)
    plot(t,d.s.x(4,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('speed (m/s)')
    title('$v$ (car speed) (state 4)')
    
    subplot(3,3,5)
    plot(t,d.s.x(5,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('angle (rad)')
    title('$\alpha_T$ (sideslip angle) (state 5)')
    
    subplot(3,3,6)
    plot(d.s.x(6,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('angular velocity (rad/s)')
    title('$\dot{\psi}$ (yaw angle velocity) (state 6)')
    
    subplot(3,3,7)
    plot(d.s.u(1,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('angle (rad)')
    title('$\delta$ (steering angle) (control input 1)')
    
    subplot(3,3,8)
    plot(d.s.u(2,1:d.p.k_final))
    xlabel('time (s)')
    ylabel('force (N)')
    title('$F$ (engine force) (control input 2)')
    
    subplot(3,3,9)
    plot(d.s.x(1,1:d.p.k_final),d.s.x(2,1:d.p.k_final))
    xlabel('position (m)')
    ylabel('position (m)')
    title('position on $x$-$y$ plane')
    
end
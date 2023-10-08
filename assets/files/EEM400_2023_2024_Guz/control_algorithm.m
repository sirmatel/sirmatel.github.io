function d = control_algorithm(d,k)
    
    % distances to left and right walls
    d.s.dist_left(k,1) = norm(d.p.checkpoints(d.p.closest_cp(k,1)).left - ...
        d.s.x(1:2,k),2);
    d.s.dist_right(k,1) = norm(d.p.checkpoints(d.p.closest_cp(k,1)).right - ...
        d.s.x(1:2,k),2);
    
    % lane keeping error
    e_lane = d.s.dist_left(k,1) - d.s.dist_right(k,1);
    
    % speed error
    v_ref = 8;
    e_speed = v_ref - d.s.x(4,k);
    
    % P-controller gain for lane keeping control
    d.p.K_lane = 0.5;
    
    % P-controller gain for speed control
    d.p.K_speed = 100;
    
    %% compute control inputs
    
    % steering angle = "K_lane" * "lane keeping error"
    d.s.u(1,k) = d.p.K_lane*e_lane;
    
    % engine force = "K_speed" * "speed error"
    d.s.u(2,k) = d.p.K_speed*e_speed;
    
end
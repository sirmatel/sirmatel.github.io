function d = main_car_sim()
    
    dbstop if error
    
    % animation toggle
    %d.p.animation = 0; % animation off
    d.p.animation = 1; % animation on
    
    d = build_parameters(d);
    
    d = build_signals(d);
    
    initialize_animation(d);
    
    for k = 1:d.p.k_max
        
        % find index of closest checkpoint
        [~,d.p.closest_cp(k,1)] = min(sum(([d.p.checkpoints.center] - ...
            repmat(d.s.x(1:2,k),1,length(d.p.checkpoints))).^2));
        
        d = control_algorithm(d,k);
        
        d = check_car_condition(d,k);
        
        animate_car_motion(d,k);
        
        d = simulate_car_motion(d,k);
        
        if k > 1
            
            if d.p.closest_cp(k-1,1) == 600 && d.p.closest_cp(k,1) == 1
                
                disp('Car has finished the race successfully!')
                
                disp(['Elapsed time is ',num2str(d.p.total_time),' seconds.'])
                
                break
                
            end
            
        end
        
    end
    
end

function d = build_parameters(d)
    
    % sampling time
    d.p.T = 0.1; % unit: seconds
    
    % minimum allowed speed
    d.p.v_min = 0.1; % unit: m/s
    
    % simulation length
    d.p.k_max = 5000;
    % in number of time steps
    
    % state dimension
    d.p.n_x = 6;
    
    % input dimension
    d.p.n_u = 2;
    
    % initial state
    d.p.x0 = [1.5 0 0 1 0 0]';
    
    % race time counter
    d.p.total_time = 0;
    
    % build race track parameters
    d = testTrack1(d);
    
    % gravitational acceleration
    d.c.g = 9.81; % unit: m/s^2
    
    % physical parameters of the car
    d.c.mF0 = 700;
    d.c.mR0 = 600;
    d.c.m = d.c.mF0 + d.c.mR0;
    d.c.I = 10000;
    d.c.lT = 3.5;
    d.c.a = (d.c.mR0/d.c.m)*d.c.lT;
    d.c.b = d.c.lT - d.c.a;
    d.c.nF = 2;
    d.c.nR = 2;
    d.c.muy = 0.8;
    
end

function d = testTrack1(d)

    trackWidth = 7;
    
    d.p.trackWidth = trackWidth;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];

    checkpoints = add_turn(checkpoints, 0, 76, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 50, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 8, trackWidth);
    checkpoints = add_turn(checkpoints, -0.1, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.1, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.5, 15, trackWidth);
    checkpoints = add_turn(checkpoints, -0.5, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.5, 15, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 60, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 10, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 20, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 55, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 90.3, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 5.3, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 20, trackWidth);
    
    checkpoints = checkpoints(2:end);
    
    d.p.checkpoints = checkpoints;
    
end

function checkpoints = add_turn(checkpoints, phi, L, width)

    kappa = (phi*(2*pi))/L;
    N = 40;
    ds = L / N;
    
    for i=1:N
        
        checkpoints(end+1).yaw = checkpoints(end).yaw + kappa * ds;
        c = cos(checkpoints(end).yaw);
        s = sin(checkpoints(end).yaw);
        f = [c;s];
        n = [-s;c];
        checkpoints(end).center = checkpoints(end-1).center + f * ds;
        checkpoints(end).left = checkpoints(end).center + n * width/2;
        checkpoints(end).right = checkpoints(end).center - n * width/2;
        checkpoints(end).forward_vector = f;
        
    end

end

function d = build_signals(d)
    
    % state trajectory
    d.s.x = NaN(d.p.n_x,d.p.k_max+1);
    
    % initialize state trajectory
    % from initial state
    d.s.x(:,1) = d.p.x0;
    
    % control input trajectory
    d.s.u = NaN(d.p.n_u,d.p.k_max);
    
    % closest checkpoint record
    d.p.closest_cp = NaN(d.p.k_max,1);
    
end

function initialize_animation(d)
    
    if d.p.animation == 1
        
        figure(gcf)
        clf
        set(gcf,'color',[1 1 1]);
        hold on
        left_points = [d.p.checkpoints(:).left];
        right_points = [d.p.checkpoints(:).right];
        forward_vectors = [d.p.checkpoints.forward_vector];
        pts=[fliplr(right_points) ...
            right_points(:,end) ...
            left_points ...
            left_points(:,1)];
        fill(pts(1,:), pts(2,:),[1 1 1]*.8,'EdgeAlpha',0)
        width = .6;
        normals = width*[0 -1;1 0]*forward_vectors;
        left_points = left_points + normals;
        right_points = right_points - normals;
        plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'k','LineWidth',1);
        plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'k','LineWidth',1);
        bounds = [min([left_points right_points]');
            max([left_points right_points]')];
        xlim(bounds(:,1))
        ylim(bounds(:,2))
        axis off
        set(gca, 'Position', [0 0 1 1])
        xlim(mean(xlim) + diff(xlim) * [-1 1] * 0.6)
        ylim(mean(ylim) + diff(ylim) * [-1 1] * 0.6)
        daspect([1 1 1])
        
    end
    
end

function d = check_car_condition(d,k)
    
    dist_max = max(d.s.dist_left(k,1),d.s.dist_right(k,1));
    
    if dist_max > d.p.trackWidth
       
        error('car has crashed into wall!')
        
    end
    
    if d.s.x(4,k) < d.p.v_min
       
        error('car got stuck with almost zero speed!')
        
    end
    
end

function animate_car_motion(d,k)
    
    if d.p.animation == 1
        
        px = d.s.x(1,k);
        py = d.s.x(2,k);
        vx = d.s.x(4,k)*cos(d.s.x(3,k));
        vy = d.s.x(4,k)*sin(d.s.x(3,k));
        
        d = [vx;vy];
        d = d / norm(d);
        R = [d [-d(2); d(1)]];
        vehicleRectangle = R*[1 0;0 .5]*[1 1 -1 -1;1 -1 -1 1] + ...
            repmat([px;py],1,4);
        fill(vehicleRectangle(1,:),vehicleRectangle(2,:),[1 0 0]);
        daspect([1 1 1])
        
        drawnow
        
    end
    
end

function d = simulate_car_motion(d,k)
    
    [~,x_next_full] = ode45(@(t,x) ...
        car_equations_of_motion(t,x,d.s.u(:,k),d),...
        linspace(0,d.p.T,10),d.s.x(:,k));
    
    d.s.x(:,k+1) = x_next_full(end,:)';
   
    d.p.total_time = d.p.total_time + d.p.T;
    
end

function dxdt = car_equations_of_motion(t,x,u,d)
    
    %X = x(1);
    %Y = x(2);
    PSI = x(3);
    v = x(4);
    ALPHAT = x(5);
    dPSI = x(6);
    
    DELTA = u(1);
    
    FxF = u(2)/2;
    FxR = u(2)/2;
    
    ALPHAF = atan2((v * sin(ALPHAT) + d.c.a * dPSI), ...
        (v * cos(ALPHAT))) - DELTA;
    ALPHAR = atan2((v * sin(ALPHAT) - d.c.b * dPSI), ...
        (v * cos(ALPHAT)));
    
    FzF = d.c.mF0 * d.c.g;
    FzR = d.c.mR0 * d.c.g;
    
    FyF = d.c.nF * CharacteristicPacejka(ALPHAF, FzF/d.c.nF, d.c.muy);
    FyR = d.c.nR * CharacteristicPacejka(ALPHAR, FzR/d.c.nR, d.c.muy);
    
    dxdt(1,1) = v * cos(ALPHAT + PSI);
    dxdt(2,1) = v * sin(ALPHAT + PSI);
    dxdt(3,1) = dPSI;
    dxdt(4,1) = (FxF * cos(ALPHAT - DELTA) + FxR * cos(ALPHAT) + ...
        FyF * sin(ALPHAT - DELTA) + FyR * sin(ALPHAT))/d.c.m;
    dxdt(5,1) = ( - FxF * sin(ALPHAT - DELTA) - FxR * sin(ALPHAT) + ...
        FyF * cos(ALPHAT - DELTA) + ...
        FyR * cos(ALPHAT) - d.c.m * v * dPSI) / (d.c.m * v);
    dxdt(6,1) = (FxF * d.c.a * sin(DELTA) + ...
        FyF * d.c.a * cos(DELTA) - FyR * d.c.b) / d.c.I;
    
end

function Fy = CharacteristicPacejka(alpha, Fz, muy)
    
    % Input
    % alpha - slip angle [rad]
    % Fz    - Load [N]
    % muy   - Lateral friction coefficient (*1000) [-]
    
    self.a0 = 1;
    self.a1 = 0;
    self.a2 = 800;
    self.a3 = 3000;
    self.a4 = 50;
    self.a5 = 0;
    self.a6 = 0;
    self.a7 = -1;
    self.a8 = 0;
    self.a9 = 0;
    self.a10 = 0;
    self.a11 = 0;
    self.a12 = 0;
    self.a13 = 0;
    
    % Slip angle treatment
    ALPHA = asin(sin(alpha)); % [rad]
    ALPHA = 180 / pi * ALPHA; % Conversion [rad] - [deg]
    
    % Nominal parameters
    a0 = self.a0;
    a1 = self.a1;
    a2 = self.a2;
    a3 = self.a3;
    a4 = self.a4;
    a5 = self.a5;
    a6 = self.a6;
    a7 = self.a7;
    a8 = self.a8;
    a9 = self.a9;
    a10 = self.a10;
    a11 = self.a11;
    a12 = self.a12;
    a13 = self.a13;
    
    Fz = Fz/1000;           % Conversion [N] - [kN]
    
    camber = 0;             % Camber angle
    
    C = a0;                 % Shape factor
    muy0 = a1 * Fz + a2;      % Lateral friction coefficient nominal [-]
    muy = muy * 1000;         % Lateral friction coefficient operacional
    D = muy0 * Fz;            % muy = lateral friction coefficient
    BCD = a3 * sin(2 * atan(Fz/a4))*(1-a5 * abs(camber)); % Cornering stiffness
    E = a6 * Fz + a7;         % Curvature factor
    B = BCD/(C * D);          % stiffness factor
    Sh = a8 * camber + a9 * Fz + a10;      % Horizontal shift
    Sv = a11 * Fz * camber + a12 * Fz + a13; % Vertical shift
    ALPHAeq = muy0/muy*(ALPHA + Sh);   % Equivalent slip angle
    
    % Reference characteristics
    fy = D * sin(C * atan(B * ALPHAeq - E*(B * ALPHAeq - atan(B * ALPHAeq))));
    
    % Lateral force
    Fy = -muy/muy0*(fy + Sv);
    
end
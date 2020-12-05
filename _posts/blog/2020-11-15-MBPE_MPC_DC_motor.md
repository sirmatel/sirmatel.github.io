---
layout: single
category: blog
author_profile: true
title: Model-based parameter estimation and model predictive control (tracking) of a DC motor using Arduino, MATLAB, and YALMIP
tags: [system identification,model-based parameter estimation,control,tracking,linear MPC,hardware implementation]
comments: true
header:
  teaser: "MBPE_MPC_DC_motor.png"
date: '2020-11-15'
sidebar:
  nav: "blog"
---

In this post we will attempt to create a feedback position control system for a DC motor using Arduino and model-based methods of control engineering. In particular, we will use model-based parameter estimation (MBPE) for system identification, and model predictive control (MPC) for solving the tracking problem (i.e., making the state follow a given reference trajectory). We will need MATLAB, <a href="https://yalmip.github.io/" style="color: #2d5a8c">YALMIP</a>[^Lofberg2004] (a free Octave/MATLAB toolbox for optimization modeling), <a href="https://github.com/coin-or/Ipopt" style="color: #2d5a8c">Ipopt</a>[^Waechter2006] (for solving the resulting convex quadratic optimization problems), and an Arduino board that is connected to a computer running MATLAB.

We start by building the physical setup, for which we will be using the following (the exact components that were used are given in the links, however equivalents may also be used):

1) a single-board microcontroller (we use Arduino Uno <a href="https://store.arduino.cc/arduino-uno-rev3" style="color: #2d5a8c">https://store.arduino.cc/arduino-uno-rev3</a>),

2) a DC motor outfitted with a quadrature encoder (we use LEGO NXT Interactive Servo Motor <a href="https://brickset.com/sets/9842-1/Interactive-Servo-Motor" style="color: #2d5a8c">https://brickset.com/sets/9842-1/Interactive-Servo-Motor</a> and its cable <a href="https://brickset.com/sets/8529-1/Connector-Cables-for-Mindstorms-NXT" style="color: #2d5a8c">https://brickset.com/sets/8529-1/Connector-Cables-for-Mindstorms-NXT</a>),

3) an H-Bridge (we use L9110S Dual-Channel H-Bridge Motor Driver Module <a href="https://www.makerlab-electronics.com/product/l9110s-dual-channel-h-bridge-motor-driver-module/" style="color: #2d5a8c">https://www.makerlab-electronics.com/product/l9110s-dual-channel-h-bridge-motor-driver-module/</a>),

4) a small breadboard,

5) a 9V battery,

6) wires.

A sketch of the physical setup (created using <a href="https://fritzing.org/" style="color: #2d5a8c">Fritzing</a> and <a href="https://www.leocad.org/" style="color: #2d5a8c">LeoCAD</a>), showing the components and wire connections, can be seen in the figure below.
![MBPE and MPC of DC motor, physical setup]({{ site.url }}/images/MBPE_MPC_DC_motor_setup.jpg){: .center-image }

Note that one end of the LEGO NXT cable must be stripped to gain access to its 6 individual wires (see here for details: <a href="https://www.instructables.com/How-to-use-LEGO-NXT-sensors-and-motors-with-a-non-/" style="color: #2d5a8c">https://www.instructables.com/How-to-use-LEGO-NXT-sensors-and-motors-with-a-non-/</a>).

To be able to use model-based methods, we derive a discrete-time linear model for the DC motor, with pulse-width modulation (PWM) duty cycle as input, and angular position and velocity as the state variables, as follows:

$$
\begin{equation}
\begin{bmatrix}
    x_1(t+1) \\
	x_2(t+1)
  \end{bmatrix} = \begin{bmatrix}
    1 \quad T_s \\
	p_1 \quad p_2
  \end{bmatrix} \begin{bmatrix}
    x_1(t) \\
	x_2(t)
  \end{bmatrix} + \begin{bmatrix}
    0 \\
	p_3
  \end{bmatrix} u(t),
\end{equation}
$$

where $$t \in \mathbb{Z}_{\ge 0}$$ is the discrete time, $$x_1 \in \mathbb{R}$$ is the state representing angular position, $$x_2 \in \mathbb{R}$$ is the state representing angular velocity, $$T_s$$ is the sampling time, $$p_i \in \mathbb{R}$$ ($$i \in \{1,2,3\}$$) are the model parameters, while $$u(t) \in \mathbb{R}$$ is the control input representing PWM duty cycle. The physical intuition behind the model is that, the angular position is simply the angular velocity integrated over time, while we are assuming that angular velocity can be affected by both state variables (due to effects such as viscous damping) and can be directly actuated by the PWM duty cycle. Note that, for simplicity, we are ignoring the electrical dynamics and effects such as friction.

To get the estimates of the model parameters $$p_i$$, we can do a system identification experiment using the following code (download from here: <a href="https://sirmatel.github.io/assets/files/MBPE_MPC_DC_motor_conduct_SI_experiment.m" style="color: #2d5a8c; text-decoration:underline">MBPE_MPC_DC_motor_conduct_SI_experiment.m</a>):
````matlab
function [x,u,T_s] = MBPE_MPC_DC_motor_conduct_SI_experiment()
    
    % create an Arduino object
    ardn = arduino('COM3','Uno','Libraries','RotaryEncoder');
    
    % connect to the encoder
    % (pins D2 and D3 of Arduino board; 180 pulses per revolution)
    ppr_value = 180;
    encoder = rotaryEncoder(ardn,'D2','D3',ppr_value);
    
    % define pins D5 and D6 of Arduino board for DC motor connection
    motor1_D_A = 'D5';
    motor1_D_B = 'D6';
    
    % sampling time (seconds)
    T_s = 0.15;
    
    % length of system identification experiment
    % (in number of time steps)
    t_max = 101;
    
    % preallocate memory for signals
    PWM_duty_cycle = NaN(t_max,1);
    theta = NaN(t_max,1);
    omega = NaN(t_max,1);
    
    CPU_time_full = NaN(t_max,1);
    CPU_time_active = NaN(t_max,1);
    
    % stop motor and reset encoder count
    writePWMDutyCycle(ardn, motor1_D_A, 0)
    writePWMDutyCycle(ardn, motor1_D_B, 0)
    pause(3)
    resetCount(encoder)
    
    % choose amplitude of PWM duty cycle input
    u_amplitude = 0.4; % maximum: 1
    
    % choose switching threshold in angular position
    theta_switch_threshold = 120;
    
    % conduct system identification experiment
    for t = 1:t_max
        
        t_full = tic;
        
        t_active = tic;
        
        % get angular position measurement from encoder
        theta(t) = readCount(encoder);
        
        % get angular velocity measurement from encoder
        omega(t) = readSpeed(encoder);
        
        % calculate PWM duty cycle to be applied
        if t == 1
            
            PWM_duty_cycle(t) = u_amplitude;
            
        else
            
            if theta(t) > theta_switch_threshold
                
                PWM_duty_cycle(t) = -u_amplitude;
                
            elseif theta(t) < -theta_switch_threshold
                
                PWM_duty_cycle(t) = u_amplitude;
                
            else
                
                PWM_duty_cycle(t) = PWM_duty_cycle(t-1);
                
            end
            
        end
        
        % apply calculated PWM duty cycle to the DC motor
        if PWM_duty_cycle(t) > 0
            
            writePWMDutyCycle(ardn, motor1_D_A, PWM_duty_cycle(t))
            writePWMDutyCycle(ardn, motor1_D_B, 0)
            
        else
            
            writePWMDutyCycle(ardn, motor1_D_A, 0)
            writePWMDutyCycle(ardn, motor1_D_B, abs(PWM_duty_cycle(t)))
            
        end
        
        % record real time spent for computations and communications
        % during current time step
        CPU_time_active(t) = toc(t_active);
        
        % add pausing time to ensure each time step is
        % close to the chosen sampling time
        if CPU_time_active(t) < T_s
            
            pause(T_s - CPU_time_active(t))
            
        end
        
        % record total real time spent during current time step
        CPU_time_full(t) = toc(t_full);
        
    end
    
    % stop motor
    writePWMDutyCycle(ardn, motor1_D_A, 0)
    writePWMDutyCycle(ardn, motor1_D_B, 0)
    
    % clear Arduino objects
    clear encoder ardn
    
    % convert angular position to radian
    theta_rad = theta*(2*pi)/(4*ppr_value);
    
    % convert angular velocity to radian/s
    omega_rad_p_s = omega*(2*pi)/60;
    
    % record state trajectory as x
    x = [theta_rad omega_rad_p_s];
    
    % record input trajectory as u
    u = PWM_duty_cycle;
    
end
````
This code alternates the sign of the PWM duty cycle input $$u(t)$$ by switching it each time the angular position reaches the selected threshold value, resulting in a rectangular wave pattern for $$u(t)$$. The results of the experiment can be seen in the figure below.
![MBPE and MPC of DC motor, SI experiment]({{ site.url }}/images/MBPE_MPC_DC_motor_SI_experiment.png){: .center-image }

Having thus gathered data necessary for conducting system identification, we continue by using the data for estimating the parameters of the DC motor state space model via the following code implementing MBPE (download from here: <a href="https://sirmatel.github.io/assets/files/MBPE_MPC_DC_motor_conduct_MBPE.m" style="color: #2d5a8c; text-decoration:underline">MBPE_MPC_DC_motor_conduct_MBPE.m</a>):
````matlab
function [A,B] = MBPE_MPC_DC_motor_conduct_MBPE(x,u,T_s)
    
    % assign data as system identification data
    data = iddata(x, u, T_s, 'Name', 'DC-motor');
    
    % give names and units to data
    data.InputName = 'PWM';
    data.InputUnit = 'duty cycle';
    data.OutputName = {'Angular position', 'Angular velocity'};
    data.OutputUnit = {'rad', 'rad/s'};
    
    % configure starting time and give name to time unit
    data.Tstart = 0;
    data.TimeUnit = 's';
    
    % select guesses for parameters
    p_init = [-0.2 0.7 4]';
    
    % create an initial system with identifiable parameters
    % and the dynamical model given in DC_motor_3_parameters.m
    init_sys = idgrey('DC_motor_3_parameters',p_init,'d');
    
    % conduct model-based parameter estimation (MBPE)
    % (i.e., grey-box model estimation)
    sys = greyest(data,init_sys);
    
    % compare the experiment data with model output
    % using the estimated parameters
    opt = compareOptions('InitialCondition','zero');
    compare(data,sys,Inf,opt)
    
    % record matrices of the identified state space model
    A = sys.A;
    B = sys.B;
    
end

function [A,B,C,D] = DC_motor_3_parameters(p,T_s)
    
    % define discrete-time linear model of a DC motor
    % with three identifiable parameters in state space form
    
    A = [1 T_s;
        p(1) p(2)];
    
    B = [0;p(3)];
    
    C = eye(2);
    
    D = [0;0];
    
end
````
Running this in MATLAB, the grey-box model estimation procedure results in the following state space model:

$$
\begin{equation}
\begin{bmatrix}
    x_1(t+1) \\
	x_2(t+1)
  \end{bmatrix} = \begin{bmatrix}
    1 \qquad 0.15 \\
	-0.17 \quad 0.58
  \end{bmatrix} \begin{bmatrix}
    x_1(t) \\
	x_2(t)
  \end{bmatrix} + \begin{bmatrix}
    0 \\
	5.74
  \end{bmatrix} u(t).
\end{equation}
$$

Having thus developed a discrete-time linear model for the DC motor and estimated its parameters, we can continue with constructing an MPC controller. We consider the following linear MPC formulation for the tracking problem, where the goal is to track a given angular position reference:

$$
\begin{aligned}
\text{minimize} & \quad \sum_{k=1}^{N}{\left\lVert x(k+1) - r(k+1) \right\rVert^2_Q + \left\lVert u(k) \right\rVert^2_R,}\\
\text{subject to} & \quad x(1) = \hat{x}(t) \\
& \quad \text{for } k = [1, \ldots, N]: \\
& \qquad x(k+1) = A x(k) + B u(k) \\
& \qquad x_{\text{min}} \leq x(k+1) \leq x_{\text{max}} \\
& \qquad u_{\text{min}} \leq u(k) \leq u_{\text{max}},
\end{aligned}
$$

where $$Q$$ and $$R$$ are weighting matrices defining the stage cost, $$\hat{x}(t)$$ is the measurement of the state at the current time step $$t$$, $$x_{\text{min}}$$ and $$x_{\text{max}}$$ are state constraints, whereas $$u_{\text{min}}$$ and $$u_{\text{max}}$$ are control input constraints.

We can implement the above MPC formulation by creating an MPC controller object using YALMIP via the following code (download from here: <a href="https://sirmatel.github.io/assets/files/MBPE_MPC_DC_motor_create_MPC.m" style="color: #2d5a8c; text-decoration:underline">MBPE_MPC_DC_motor_create_MPC.m</a>):
````matlab
function MPC_controller = MBPE_MPC_DC_motor_create_MPC(A,B,Q,R,N)
    
    % define control inputs as decision variables
    u = sdpvar(1,N,'full');
    
    % define states as decision variables
    x = sdpvar(2,N+1,'full');
    
    % define angular position reference as decision variables
    r = sdpvar(1,N+1,'full');
    
    % initialize objective function
    obj = 0;
    
    % initialize constraints
    con = [];
    
    for k = 1:N
        
        % ensure continuity of state trajectory
        % (i.e., direct multiple shooting)
        con = [con, x(:,k+1) == A*x(:,k) + B*u(:,k)];
        
    end
    
    for k = 1:N+1
        
        % add stage cost for states to objective function
        obj = obj + (x(1,k) - r(1,k))'*Q*(x(1,k) - r(1,k));
        
    end
    
    for k = 1:N
        
        % add stage cost for inputs to objective function
        obj = obj + u(:,k)'*R*u(:,k);
        
    end
    
    for k = 1:N
        
        % control input constraints
        con = [con, -1 <= u(:,k) <= 1];
        
        con = [con, -4 <= x(2,k+1) <= 4];
        
    end
    
    % define optimization settings
    ops = sdpsettings;
    ops.verbose = 0; % configure level of info solver displays
    ops.solver = 'ipopt'; % choose solver
    ops.ipopt.max_cpu_time = 0.01;
     
    % configure the inputs of the controller object
    % (we choose the predicted control input and state trajectories)
    controller_inputs = [x(:,1);r(:)];
    
    % configure the outputs of the controller object
    % (we choose the predicted control input and state trajectories)
    controller_outputs = [u(:);x(:)];
    
    % create MPC controller object
    MPC_controller = optimizer(con,obj,ops,...
        controller_inputs,...
        controller_outputs);
    
end
````

We can finally conduct a closed-loop system experiment, where we try to make the DC motor track an angular position reference using the above MPC controller, which we implement via the following code (download from here: <a href="https://sirmatel.github.io/assets/files/MBPE_MPC_DC_motor_conduct_tracking_experiment.m" style="color: #2d5a8c; text-decoration:underline">MBPE_MPC_DC_motor_conduct_tracking_experiment.m</a>):
````matlab
% system model
A = [1.00 0.15;
    -0.17 0.58];
B = [0;5.74];

% sampling time
T_s = 0.15;

% choose MPC parameters
N = 20; % prediction horizon
Q = 1; % angular position tracking error weighting
R = 0.1; % control input weighting

% create MPC controller object
MPC_controller = MBPE_MPC_DC_motor_create_MPC(A,B,Q,R,N);

% create an Arduino object
ardn = arduino('COM3','Uno','Libraries','RotaryEncoder');

% connect to the encoder
% (pins D2 and D3 of Arduino board; 180 pulses per revolution)
ppr_value = 180;
encoder = rotaryEncoder(ardn,'D2','D3',ppr_value);

% define pins D5 and D6 of Arduino board for DC motor connection
motor1_D_A = 'D5';
motor1_D_B = 'D6';

% length of tracking experiment
% (in number of time steps)
t_max = 101;

% preallocate memory for signals
x = NaN(2,t_max); % state (angular position and velocity)
u = NaN(1,t_max); % control input (PWM duty cycle)
u_predicted = NaN(N,t_max);
x_predicted = NaN(2*(N+1),t_max);

% create angular position reference signal
theta_ref = (2/3)*pi*square(linspace(0,3*pi,t_max+1));
theta_ref = [theta_ref(1:t_max) (2/3)*pi*ones(1,N)];

% preallocate memory for CPU time records
CPU_time_MPC = NaN(t_max,1);
CPU_time_full = NaN(t_max,1);
CPU_time_active = NaN(t_max,1);

% stop motor and reset encoder count
writePWMDutyCycle(ardn, motor1_D_A, 0)
writePWMDutyCycle(ardn, motor1_D_B, 0)
pause(3)
resetCount(encoder)

% conduct tracking experiment
for t = 1:t_max
    
    t_full = tic;
    
    t_active = tic;
    
    % get angular position measurement from encoder
    theta = readCount(encoder);
    
    % get angular velocity measurement from encoder
    omega = readSpeed(encoder);
    
    % convert angular position to radian
    theta = theta*(2*pi)/(4*ppr_value);
    
    % convert angular velocity to radian/s
    omega = omega*(2*pi)/60;
    
    % record state
    x(:,t) = [theta;omega];
    
    t_MPC = tic;
    
    % get angular position trajectory for the prediction horizon
    theta_ref_trajectory_for_k = theta_ref(1,t:(t+N));
    
    % solve MPC problem and record solution
    sol_MPC = MPC_controller([x(:,t);theta_ref_trajectory_for_k(:)]);
    
    % record predicted input and state trajectories
    u_predicted(:,t) = sol_MPC(1:N);
    x_predicted(:,t) = sol_MPC(N+1:end);
    
    % record control input to be applied at current time step
    u(:,t) = u_predicted(1,t);
    
    % record real time spent for MPC computation
    CPU_time_MPC(t) = toc(t_MPC);
    
    % apply calculated PWM duty cycle to the DC motor
    if u(:,t) > 0
        
        writePWMDutyCycle(ardn, motor1_D_A, u(:,t))
        writePWMDutyCycle(ardn, motor1_D_B, 0)
        
    else
        
        writePWMDutyCycle(ardn, motor1_D_A, 0)
        writePWMDutyCycle(ardn, motor1_D_B, abs(u(:,t)))
        
    end
    
    % record real time spent for computations and communications
    % during current time step
    CPU_time_active(t) = toc(t_active);
    
    % add pausing time to ensure each time step is
    % close to the chosen sampling time
    if CPU_time_active(t) < T_s
        
        pause(T_s - CPU_time_active(t))
        
    end
    
    % record total real time spent during current time step
    CPU_time_full(t) = toc(t_full);
    
end

% stop motor
writePWMDutyCycle(ardn, motor1_D_A, 0)
writePWMDutyCycle(ardn, motor1_D_B, 0)

% clear Arduino objects
clear encoder ardn
````
Running this in MATLAB, we get the results of the tracking experiment on the closed loop DC motor position control system, which can be seen in the figure below.
![MBPE and MPC of DC motor, tracking experiment]({{ site.url }}/images/MBPE_MPC_DC_motor_tracking_experiment.png){: .center-image }

Although the control system we created is performing reasonably well, there are many aspects that could be greatly improved, such as:

a) The sampling time of $$0.15$$ seconds is rather large, which we had to choose here due to the fact that reading the encoder measurements and solving the MPC problem are taking considerable amount of CPU time. If these could be reduced, it would be possible to choose smaller sampling times. The CPU time for MPC can be reduced by using a faster solver or MPC toolboxes that are better suited to real time operations.

b) There are some oscillations and small steady state errors in the angular position trajectory, which could be remedied by better tuning the MPC controller (by choosing different weights, prediction horizon, or even a better modeling and/or identification), and considering offset-free MPC formulations to get rid of the steady state error.

c) There are some violations of the state constraints on the angular velocity. Resulting from plant/model mismatch, this could be addressed by improving the modeling and/or identification, or using robust MPC formulations.

[^Lofberg2004]: Löfberg, J. (2004, September). YALMIP: A toolbox for modeling and optimization in MATLAB. In 2004 IEEE international conference on robotics and automation (pp. 284-289).

[^Waechter2006]: Wächter, A., & Biegler, L. T. (2006). On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming. Mathematical programming, 106(1), 25-57.

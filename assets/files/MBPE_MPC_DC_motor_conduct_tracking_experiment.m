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
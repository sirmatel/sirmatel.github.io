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
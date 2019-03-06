---
layout: single
category: tutorial
author_profile: false
title: Implementing nonlinear model predictive control in MATLAB with MPCTools
tags: [control,nonlinear MPC,simulation]
comments: true
header:
  teaser: "NMPC_MPCTools.jpg"
date: '2019-03-06'
sidebar:
  nav: "tutorials"
---

In this tutorial we will attempt to create nonlinear model predictive control (MPC) code in MATLAB using MPCTools. We will need MATLAB (version R2015b or higher), <a href="https://bitbucket.org/rawlings-group/octave-mpctools/overview" style="color: #2d5a8c; text-decoration:underline">MPCTools</a>[^Risbeck2016] (a free Octave/MATLAB toolbox for nonlinear MPC), and <a href="https://web.casadi.org/" style="color: #2d5a8c; text-decoration:underline">CasADi</a>[^Andersson2018] (version 3.1 or higher) (a free Python/MATLAB toolbox for nonlinear optimization and numerical optimal control).

We consider the following nonlinear MPC formulation:

$$
\begin{aligned}
\text{minimize} & \quad \int_{t}^{t+T_p}{l(x(\tau),u(\tau))d\tau} + V_f(x(t+T_p)) \\
\text{subject to} & \quad x(t) = \hat{x}(t) \\
& \quad \text{for } \tau \in [t, t+T_p]: \\
& \qquad \dot{x}(\tau) = f(x(\tau),u(\tau)) \\
& \qquad x_{\text{min}} \leq x(\tau) \leq x_{\text{max}} \\
& \qquad u_{\text{min}} \leq u(\tau) \leq u_{\text{max}} \\
& \quad e_f(x(t+T_p)) \leq 0,
\end{aligned}
$$

where $$T_p$$ is the prediction horizon (in time units), $$l(\cdot)$$ is the stage cost, $$x \in \mathbb{R}^n$$ is the state vector, $$u \in \mathbb{R}^m$$ is the control input vector, $$V_f(\cdot)$$ is the terminal cost, $$\hat{x}(t)$$ is the measurement, $$f(\cdot)$$ is the dynamics, $$x_{\text{min}}$$ and $$x_{\text{max}}$$ are state constraints, $$u_{\text{min}}$$ and $$u_{\text{max}}$$ are control input constraints, while $$e_f$$ is the terminal state constraint.

As an example, we take the following two dimensional system[^Chen1998]:

$$
\begin{aligned}
\dot{x}_1(t) & = x_2(t) + u(t) \left( \mu + \left( 1 - \mu \right)x_1(t) \right) \\
\dot{x}_2(t) & = x_1(t) + u(t) \left( \mu - 4 \left( 1 - \mu \right)x_2(t)\right),
\end{aligned}
$$

where $$x_1(t) \in \mathbb{R}$$ and $$x_2(t) \in \mathbb{R}$$ are the state variables, $$u(t) \in \mathbb{R}$$ is the control input, and $$\mu$$ is a parameter we assume to be $$0.5$$ here. In compact form we can write the dynamics as $$\dot{x}(t)=f(x(t),u(t))$$, which we can define in code as follows:
````matlab
function dxdt = define_dynamics(x, u)
    
    dxdt = [x(2) + u*(0.5 + 0.5*x(1));
        x(1) + u*(0.5 - 2*x(2))];
    
end
````

We specify various parameters and pre-allocate memory for the signals as follows:
````matlab
function d = build_setup(d)
    
    % state constraints
    d.p.x_min = -Inf;
    d.p.x_max = Inf;
    
    % control input constraints
    d.p.u_min = -2;
    d.p.u_max = 2;
    
    % NMPC prediction horizon (in number of time steps)
    d.p.N_NMPC = 15;
    
    % sampling time (in time units)
    d.p.T = 0.1;
    
    % number of state variables
    d.p.n_x = 2;
    
    % number of control inputs
    d.p.n_u = 1;
    
    % simulation length (in number of time steps)
    d.p.t_final = 120;
    
    % pre-allocate memory
    d.s.x = NaN(d.p.n_x,d.p.t_final);
    d.s.u = NaN(d.p.n_u,d.p.t_final);
    
    % set initial state
    % (d.p.x0 is an argument of the main function)
    d.s.x(:,1) = d.p.x0;
    
    % state constraints vector
    d.p.x_min_v = d.p.x_min*ones(d.p.n_x,1);
    d.p.x_max_v = d.p.x_max*ones(d.p.n_x,1);
    
    % control input constraints vector
    d.p.u_min_v = d.p.u_min*ones(d.p.n_u,1);
    d.p.u_max_v = d.p.u_max*ones(d.p.n_u,1);
    
end
````
Here the use of ````-Inf```` and ````Inf```` indicates that the state is unconstrained, while the control input is constrained as $$-2 \leq u(t) \leq 2$$.

We can create a simulator object, that will act as the plant within the simulation, as follows:
````matlab
function d = create_simulator(d)

    d.c.simulator = ...
        d.c.mpc.getCasadiIntegrator(@define_dynamics, ...
        d.p.T, [d.p.n_x, d.p.n_u], {'x', 'u'});
    
end
````
Here the arguments are the dynamics $$f(\cdot)$$, sampling time $$T$$, and dimensions of the state and the control input. Note that, in the code, the structure ````d```` is the main structure containing everything, while the fields of ````d````, namely ````p````, ````s````, and ````c```` contain parameters, signals, and controller objects, respectively.

We can build the nonlinear MPC with the following parts:

a) Import the dynamics $$f(\cdot)$$:
````matlab
ode_casadi_NMPC = d.c.mpc.getCasadiFunc(...
        @define_dynamics, ...
        [d.p.n_x, d.p.n_u], ...
        {'x', 'u'});
````

b) Specify how $f(\cdot)$ should be discretized in time to get $x(k+1) = F(x(k),u(k))$ by creating the function $F(\cdot)$:
````matlab
F = d.c.mpc.getCasadiFunc(...
        ode_casadi_NMPC, ...
        [d.p.n_x, d.p.n_u], ...
        {'x', 'u'}, ...
        'rk4', true(), ...
        'Delta', d.p.T);
````
Here the arguments are dimensions of the state and control input, whether to use an explicit Runge-Kutta method or not, via setting ````'rk4'```` either ````true```` or ````false````, and the timestep ````'Delta'````.

c) Considering a stage cost $l(\cdot)$ as follows

$$
\begin{equation}
l(x(t),u(t)) = \norm{x(t)}^2_Q + \norm{u(t)}^2_P,
\end{equation}
$$

with

$$
\begin{equation}
Q = \begin{bmatrix}
    0.5 ~ 0 \\
    0 ~ 0.5
  \end{bmatrix} \qquad R = 1,
\end{equation}
$$

we create $l(\cdot)$ in code as follows:
````matlab
function l = define_stage_cost(x,u)
    
    l = x'*[0.5 0;0 0.5]*x + u^2;
    
end
````

[^Risbeck2016]: Risbeck, M. J., & Rawlings, J. B. (2016). MPCTools: Nonlinear model predictive control tools for CasADi.

[^Andersson2018]: Andersson, J. A., Gillis, J., Horn, G., Rawlings, J. B., & Diehl, M. (2018). CasADi: a software framework for nonlinear optimization and optimal control. Mathematical Programming Computation, 1-36.

[^Chen1998]: Chen, H., & Allgöwer, F. (1998). A Quasi-Infinite Horizon Nonlinear Model Predictive Control Scheme with Guaranteed Stability. Automatica, 34(10), 1205-1217.

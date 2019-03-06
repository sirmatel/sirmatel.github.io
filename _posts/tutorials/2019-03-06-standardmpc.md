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
\text{minimize } & \quad \int_{t}^{t+T_p}{l(x(\tau),u(\tau))d\tau} + V_f(x(t+T_p)) \\
\text{subject to } & \quad x(t) = \hat{x}(t) \\
& \quad \text{for } \tau \in [t, t+T_p]: \\
& \qquad \dot{x}(\tau) = f(x(\tau),u(\tau)) \\
& \qquad x_{\text{min}} \leq x(\tau) \leq x_{\text{max}} \\
& \qquad u_{\text{min}} \leq u(\tau) \leq u_{\text{max}} \\
& \quad e_f(x(t+T_p)) \leq 0,
\end{aligned}
$$

[^Risbeck2016]: Risbeck, M. J., & Rawlings, J. B. (2016). MPCTools: Nonlinear model predictive control tools for CasADi.

[^Andersson2018]: Andersson, J. A., Gillis, J., Horn, G., Rawlings, J. B., & Diehl, M. (2018). CasADi: a software framework for nonlinear optimization and optimal control. Mathematical Programming Computation, 1-36.

[^Chen1998]: Chen, H., & Allgöwer, F. (1998). A Quasi-Infinite Horizon Nonlinear Model Predictive Control Scheme with Guaranteed Stability. Automatica, 34(10), 1205-1217.

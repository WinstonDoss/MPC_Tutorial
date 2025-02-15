%% MPC for a 1D Robot: x(k+1) = x(k) + u(k)
% The cost is
%    J = sum_{k=0}^{N-1} [ (x(k)-5)^2 + u(k)^2 ]
% with the input constraint: -2 <= u(k) <= 2.
% For a 3-step horizon (N=3), note that x0 is given so the first term is constant.
% We implement receding-horizon control (MPC) and plot the closed-loop trajectories.

clear; clc; close all;

%% Parameters
N = 3;                   % prediction horizon (number of control moves)
x_target = 5;            % desired target state
u_min = -1;              % minimum control input
u_max = 1;               % maximum control input

% The discrete-time dynamics: x(k+1) = x(k) + u(k)
% Over the horizon starting from current state x, the predicted states are:
%   x0 = x (given)
%   x1 = x + u0
%   x2 = x + u0 + u1
% The cost (ignoring the constant term (x-5)^2) becomes:
%   J = u0^2 + (x+u0-5)^2 + u1^2 + (x+u0+u1-5)^2 + u2^2.
% This is a quadratic function of the decision variables u = [u0; u1; u2].

% By expanding, one can show that the QP in the standard form
%    min  0.5*u'*H*u + f'*u
% has a constant Hessian:
H = [6, 2, 0;
     2, 4, 0;
     0, 0, 2];
% and the linear term depends on the current state x:
%    f = [4*(x-5); 2*(x-5); 0]
%
% (A detailed derivation is provided in the explanation.)

%% Simulation Setup
Tsim = 15;              % total simulation steps (closed-loop)
x = 0;                  % initial state
x_traj = zeros(Tsim+1,1); % to store state trajectory
u_traj = zeros(Tsim,1);   % to store applied control inputs
x_traj(1) = x;

% Options for quadprog (suppress output)
options = optimoptions('quadprog','Display','none');

%% MPC Loop
for t = 1:Tsim
    % Build the linear term f for the current state x:
    f = [4*(x - x_target);
         2*(x - x_target);
         0];
    
    % Define bounds for the control moves over the horizon:
    lb = u_min * ones(N,1);
    ub = u_max * ones(N,1);
    
    % Solve the QP:
    %   min  0.5*u'*H*u + f'*u
    %   s.t. lb <= u <= ub
    u_opt = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    
    % Apply only the first control input (receding horizon):
    u_applied = u_opt(1);
    u_traj(t) = u_applied;

    % Define noise standard deviation (adjust as needed)
    sigma = 0.1;  % standard deviation of the noise
    
    % Update the state using the dynamics with added noise:
    noise = sigma * randn();  % generate a random number from a normal distribution (mean 0, std sigma)
    x = x + u_applied + noise;
    x_traj(t+1) = x;
    
    % Update the state using the dynamics:
    %x = x + u_applied;
    %x_traj(t+1) = x;
end

%% Visualization
t_vec = 0:Tsim;

figure;
% Plot state trajectory
subplot(2,1,1);
stairs(t_vec, x_traj, 'b', 'LineWidth', 2); hold on;
plot(t_vec, x_target*ones(size(t_vec)), 'r--', 'LineWidth', 2);
xlabel('Time step'); ylabel('State x');
title('State Trajectory');
legend('x', 'x_{target}');
grid on;

% Plot control inputs
subplot(2,1,2);
stairs(0:Tsim-1, u_traj, 'k', 'LineWidth', 2);
xlabel('Time step'); ylabel('Control input u');
title('Control Input Trajectory');
grid on;

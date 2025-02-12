%% mpc_bicycle.m
% MPC for Vehicle Kinematics using the Discrete-Time Bicycle Model
clear; clc; close all;

%% Parameters
% Vehicle parameters
v = 10;         % constant speed (m/s)
L = 2.5;        % wheelbase (m)
T = 0.1;        % sampling time (s)

% Discrete-time model matrices
A = [1, T*v;    % = [1, 1]
     0,   1];
B = [0; T*v/L]; % = [0; 0.4]

nx = size(A,1); % number of states (2)
nu = size(B,2); % number of inputs (1)

%% MPC Parameters
N = 10;   % prediction horizon (number of steps)

% Cost weighting matrices
Q  = diag([1, 1]);   % state cost (penalize lateral error and heading error)
R  = 1;              % input cost (penalize steering action)
Qf = Q;             % terminal cost

%% Build Prediction Matrices
% We wish to predict the state evolution over the horizon:
%    x_pred = Sx*x0 + Su*U, where U = [u0; u1; ...; u_{N-1}]
Sx = zeros(nx*N, nx);
Su = zeros(nx*N, nu*N);

for i = 1:N
    Sx((i-1)*nx+1:i*nx, :) = A^i;
    for j = 1:i
        Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j) * B;
    end
end

% Build block-diagonal cost matrices for the horizon.
% Penalize predicted states with Q and terminal state with Qf.
Q_bar = blkdiag(kron(eye(N-1), Q), Qf);
R_bar = kron(eye(N), R);

%% Simulation Setup
sim_time = 5;              % total simulation time (s)
Nsim = sim_time / T;       % number of simulation steps

% Preallocate storage for states and inputs
x_history = zeros(nx, Nsim+1);
u_history = zeros(nu, Nsim);

% Initial state (e.g., small lateral offset and heading error)
x_current = [0.5; 0.05];  % [lateral position; heading error]
x_history(:,1) = x_current;

%% MPC Simulation Loop
options = optimoptions('quadprog','Display','off');

figure;
for k = 1:Nsim
    %--- Formulate the QP for the current state x_current ---
    % Predicted state trajectory: x_pred = Sx*x_current + Su*U.
    % Cost function: J = (Sx*x_current + Su*U)'*Q_bar*(Sx*x_current + Su*U) + U'*R_bar*U.
    H = 2*(Su' * Q_bar * Su + R_bar);
    f = 2*(Sx*x_current)' * Q_bar * Su;
    f = f';  % Ensure f is a column vector

    % Define bounds for the steering angle for each prediction step.
    lb = -0.5 * ones(nu*N, 1);  % lower bound: -0.5 rad for each control input
    ub =  0.5 * ones(nu*N, 1);  % upper bound:  0.5 rad for each control input
    
    % Solve QP with bounds
    [U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
    if exitflag ~= 1
        warning('QP did not converge at step %d.', k);
    end

    
    % Solve QP (no input/state constraints in this example)
    %[U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);
    %if exitflag ~= 1
    %    warning('QP did not converge at step %d.', k);
    %end
    
    %--- Apply the first control action ---
    u_current = U_opt(1);
    u_history(:, k) = u_current;
    
    % Update state: x_{k+1} = A*x_current + B*u_current
    x_next = A*x_current + B*u_current;
    x_history(:, k+1) = x_next;
    x_current = x_next;
    
    %--- Optional: Plot the predicted trajectory ---
    x_pred = Sx * x_history(:, k) + Su * U_opt;
    y_pred     = x_pred(1:nx:end);  % lateral positions
    theta_pred = x_pred(2:nx:end);  % heading errors
    t_pred = (k:k+N-1)*T;           % prediction time instants
    
    subplot(2,1,1);
    plot(t_pred, y_pred, 'b.-','LineWidth',1.5);
    hold on;
    xlabel('Time (s)');
    ylabel('Lateral Position (y)');
    title('Predicted Trajectory (Lateral Position)');
    grid on;
    
    subplot(2,1,2);
    plot(t_pred, theta_pred, 'r.-','LineWidth',1.5);
    hold on;
    xlabel('Time (s)');
    ylabel('Heading Error (\theta)');
    title('Predicted Trajectory (Heading Error)');
    grid on;
    
    pause(0.05);  % pause for visualization
end

%% Plot Closed-Loop Trajectories (States and Control)
figure;
t = 0:T:sim_time;
subplot(3,1,1);
plot(t, x_history(1,:), 'b-o','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Lateral Position (y)');
title('Closed-Loop Lateral Position');
grid on;

subplot(3,1,2);
plot(t, x_history(2,:), 'r-o','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Heading Error (\theta)');
title('Closed-Loop Heading Error');
grid on;

subplot(3,1,3);
plot(t(1:end-1), u_history, 'k-o','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Steering Angle (u)');
title('Control Input');
grid on;

%% Plot Vehicle Trajectory in (x,y) Coordinates
% Compute the global x-coordinate based on constant speed.
x_global = (0:Nsim) * T * v;  % x position at each time step
y_global = x_history(1,:);    % lateral position from the state history

figure;
plot(x_global, y_global, 'b-o','LineWidth',1.5);
hold on;
y_desired = 0;  % desired lateral position (can be changed as needed)
plot(x_global, y_desired*ones(size(x_global)), 'r--','LineWidth',1.5);
xlabel('Longitudinal Position (x) [m]');
ylabel('Lateral Position (y) [m]');
title('Vehicle Trajectory in Global Coordinates');
legend('Actual Trajectory', 'Desired y');
grid on;

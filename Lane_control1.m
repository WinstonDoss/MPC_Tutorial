%% mpc_bicycle_symbolic.m
% MPC for Vehicle Kinematics (Lane Keeping) using Symbolic Optimization
clear; clc; close all;

%% Parameters
% Vehicle parameters
v = 10;         % constant speed (m/s)
L = 2.5;        % wheelbase (m)
T = 0.1;        % sampling time (s)

%--------------------Fill your code here-----------------%
% Discrete-time model matrices
A = ??
B = ??
%---------------------END--------------------------------%

nx = size(A,1); % number of states (2)
nu = size(B,2); % number of inputs (1)

%% MPC Parameters
N = 3;   % prediction horizon (set to 3 for symbolic optimization)

% Cost weighting matrices
Q  = diag([1, 1]);   % state cost (penalize lateral error and heading error)
R  = 1;              % input cost (penalize steering action)
Qf = Q;             % terminal cost

%% Simulation Setup
sim_time = 0.1;              % total simulation time (s)
Nsim = sim_time / T;       % number of simulation steps

% Preallocate storage for states and inputs
x_history = zeros(nx, Nsim+1);
u_history = zeros(nu, Nsim);

% Initial state (e.g., small lateral offset and heading error)
x_current = [-0.5; 0.05];  % [lateral position; heading error]
x_history(:,1) = x_current;

%% MPC Simulation Loop with Symbolic Optimization
for k = 1:Nsim
    %--- Step 1: Declare symbolic control variables for the horizon
    syms u0 u1 u2 real
    
    %--- Step 2: Form the prediction equations using the dynamics.
    % Convert the current state to a symbolic vector.
    x0_sym = sym(x_current);
    
    fprintf('Predictions:')
    
    %--------------------Fill your code here-----------------%
    %Note: Use x0_sym,x1_sym,x2_sym for x0,x1,x2
    x1_sym = ??
    x2_sym = ??
    x3_sym = ??
    %---------------------END--------------------------------%
    
    %--- Step 3: Formulate the cost function over the horizon.
    % The cost function is the sum of state errors and control efforts:
    J = (x1_sym.' * Q * x1_sym) + (x2_sym.' * Q * x2_sym) + (x3_sym.' * Qf * x3_sym) ...
        + R*u0^2 + R*u1^2 + R*u2^2;
    
    %--- Step 4: Derive first-order conditions (take derivatives w.r.t. u0, u1, u2)
    fprintf('Equations from derivative:')
    eq1 = diff(J, u0) == 0
    eq2 = diff(J, u1) == 0
    eq3 = diff(J, u2) == 0
    
    % Solve the equations symbolically for the control inputs.
    fprintf('Solutions:')
    sol = solve([eq1, eq2, eq3], [u0, u1, u2])
    
    % Convert symbolic solutions to numeric values.
    u0_opt = double(sol.u0);
    % u1_opt = double(sol.u1); % Not used in receding horizon
    % u2_opt = double(sol.u2);
    
    %--- Step 5: Apply the first control action (receding horizon).
    u_current = u0_opt;
    u_history(:, k) = u_current;
    
    % Update the state using the dynamics: x(k+1) = A*x(k) + B*u_current.
    x_next = A*x_current + B*u_current;
    x_history(:, k+1) = x_next;
    
    % Prepare for the next iteration.
    x_current = x_next;
    
    % Display the computed control for this time step.
    %fprintf('Step %d: u0 = %.4f\n', k, u_current);
end

%% Plot Closed-Loop Trajectories (States and Control)
t = 0:T:sim_time;
figure;
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
x_global = (0:Nsim) * T * v;  % x position at each time step (constant speed)
y_global = x_history(1,:);    % lateral position from the state history

figure;
plot(x_global, y_global, 'b-o','LineWidth',1.5);
hold on;
y_desired = 0;  % desired lateral position (e.g., center of the lane)
plot(x_global, y_desired*ones(size(x_global)), 'r--','LineWidth',1.5);
xlabel('Longitudinal Position (x) [m]');
ylabel('Lateral Position (y) [m]');
title('Vehicle Trajectory in Global Coordinates');
legend('Actual Trajectory', 'Desired y');
grid on;

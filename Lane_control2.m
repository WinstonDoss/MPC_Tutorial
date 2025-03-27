%% mpc_bicycle_continuous_linearization.m
% MPC for Vehicle Kinematics (Lane Keeping) using
% Continuous-Time Linearization (Euler Discretization) + Symbolic MPC

clear; clc; close all;

%% Parameters
% Vehicle parameters
v  = 10;    % constant speed (m/s)
L  = 2.5;   % wheelbase (m)
DT = 0.1;   % sampling time (s)

% MPC Parameters
N = 3;  % prediction horizon

% Cost weighting matrices
Q  = diag([1, 1]);  % state cost
R  = 0.01;             % input cost
Qf = Q;             % terminal cost

%% Simulation Setup
sim_time = 5;               % total simulation time (s)
Nsim = sim_time / DT;       % number of simulation steps

% Preallocate storage for states and inputs
x_history = zeros(2, Nsim+1);  % state: [y; theta]
u_history = zeros(1, Nsim);

% Initial state (e.g., small lateral offset and heading error)
x_current = [-0.5; 0.7];  % [y; theta]
x_history(:,1) = x_current;

%% 1) Define the Continuous-Time Model Symbolically
% We treat the bicycle kinematics as:
%   dot(y)     = v * sin(theta)
%   dot(theta) = (v / L) * tan(u)
%
% z = [ y; theta ],   dot(z) = f_c(z,u).
%
% We'll linearize and then discretize using Euler's method.

% Define symbolic variables
syms y theta u real
z_sym = [y; theta];

% Continuous-time bicycle model
f_c_sym = [ v * sin(theta);
            (v / L) * tan(u) ];

% Compute continuous-time Jacobians:
%   A' = d f_c / d z,   B' = d f_c / d u
%--------------------Fill your code here-----------------%
A_prime_sym = ??   % 2x2
B_prime_sym = ??       % 2x1
%------------------------END-----------------------------%

%% 2) MPC Simulation Loop
for k = 1:Nsim
    %--- Step 2.1: Choose the operating point (tilde{z}, tilde{u})
    % We'll linearize about the current state, x_current, and nominal u_bar = 0
    z_tilde = x_current;
    u_bar   = 0;
    
    % Evaluate continuous-time Jacobians at (z_tilde, u_bar)
    A_prime_numeric = double( subs(A_prime_sym, [y, theta, u], [z_tilde(1), z_tilde(2), u_bar]) );
    B_prime_numeric = double( subs(B_prime_sym, [y, theta, u], [z_tilde(1), z_tilde(2), u_bar]) );
    
    %--- Step 2.2: Build the discrete-time linearized model
    %   A = I + dt * A'
    %   B = dt * B'
    %   C = dt * [ f_c(z_tilde, u_bar) - A'*z_tilde - B'*u_bar ]
    
    %--------------------Fill your code here-----------------%
    % Clue I is written as eye(2) and dt is DT
    A_numeric = ??      % 2x2
    B_numeric = ??               % 2x1
    %-----------------------END------------------------------%
    
    % Evaluate the continuous-time function f_c at (z_tilde, u_bar)
    f_c_numeric = double( subs(f_c_sym, [y, theta, u], [z_tilde(1), z_tilde(2), u_bar]) );
    
    C_numeric = DT * ( f_c_numeric - A_prime_numeric*z_tilde - B_prime_numeric*u_bar );
    
    %--- Step 2.3: Set Up the 3-Step Prediction (Symbolically) & Cost
    % Let u0, u1, u2 be the control inputs over the horizon.
    syms u0 u1 u2 real
    
    % Prediction:
    %   x0 = x_current
    %   x1 = A*x0 + B*u0 + C
    %   x2 = A*x1 + B*u1 + C
    %   x3 = A*x2 + B*u2 + C
    x0_sym = x_current;  % numeric, but treat as initial condition
    x1_sym = A_numeric*x0_sym + B_numeric*u0 + C_numeric;
    x2_sym = A_numeric*x1_sym + B_numeric*u1 + C_numeric;
    x3_sym = A_numeric*x2_sym + B_numeric*u2 + C_numeric;
    
    % Assume desired reference is [0;0] for all steps
    % Cost: J = x1^T Q x1 + x2^T Q x2 + x3^T Qf x3 + R*(u0^2 + u1^2 + u2^2)
    J = (x1_sym.'*Q*x1_sym) + (x2_sym.'*Q*x2_sym) + (x3_sym.'*Qf*x3_sym) ...
        + R*(u0^2 + u1^2 + u2^2);
    
    %--- Step 2.4: Derive the First-Order Conditions & Solve for (u0,u1,u2)
    eq1 = diff(J, u0) == 0;
    eq2 = diff(J, u1) == 0;
    eq3 = diff(J, u2) == 0;
    
    sol = solve([eq1, eq2, eq3], [u0, u1, u2]);
    
    % For receding horizon, we only apply the first control input
    u0_opt = double(sol.u0);
    
    %--- Step 2.5: Apply the First Control & Propagate State
    u_current = u0_opt;                % receding horizon
    u_history(k) = u_current;
    
    % Next state update using the *discrete-time linear model* at the current step:
    x_next = A_numeric*x_current + B_numeric*u_current + C_numeric;
    x_history(:, k+1) = x_next;
    
    % Prepare for next iteration
    x_current = x_next;
end

%% Plotting Results
t = 0:DT:sim_time;

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
xlabel('Time (s)'); ylabel('Steering Input (u)');
title('Control Input');
grid on;

%% Plot Vehicle Trajectory in "Global" Coordinates with Heading
x_global = (0:Nsim) * DT * v;
y_global = x_history(1,:);
theta_global = x_history(2,:); % Extract heading angles

figure;
plot(x_global, y_global, 'b-o','LineWidth',1.5); hold on;
plot(x_global, zeros(size(x_global)), 'r--','LineWidth',1.5);

% Add heading direction arrows
quiver(x_global, y_global, cos(theta_global), sin(theta_global), 0.05, 'k', 'LineWidth', 1.2,'MaxHeadSize', 0.3);

xlabel('Longitudinal Position (x) [m]');
ylabel('Lateral Position (y) [m]');
title('Vehicle Trajectory with Heading Indication');
legend('Actual Trajectory','Desired y', 'Heading Direction');
grid on;


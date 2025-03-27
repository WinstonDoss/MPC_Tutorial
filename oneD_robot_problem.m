%%
%%MPC Tutorial
%%@author: Winston Doss Marveldoss
%%
clear; clc; close all;

%% Simulation Parameters
Tsim = 1;            % total simulation steps (closed-loop)
x_target = 5;         % desired target state
x = 2;                % initial state
x_traj = zeros(Tsim+1,1); % to store state trajectory
u_traj = zeros(Tsim,1);   % to store applied control inputs
x_traj(1) = x;

%% MPC Loop: Derive control inputs symbolically at each time step
for t = 1:Tsim
    % Declare symbolic variables for control inputs
    syms u0 u1 u2 real
    
    % Represent current state and target as symbolic constants
    x_sym = sym(x);          % current state (as symbolic)
    target_sym = sym(x_target);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Step 1: Write the prediction equations:
    %   x0 = x_sym         (current state)
    %   x1 = x_sym + u0
    %   x2 = x_sym + u0 + u1
    %   x3 = x_sym + u0 + u1 + u2
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Step 2: Form the cost function over a 3-step horizon.
    % We drop the constant term (x0-target)^2 because x0 is known.
    % Cost function:
    %   J = ??
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    J = ??
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Step 3: Take the first derivative with respect to each input.
    % Set the derivatives equal to zero.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    eq1 = diff(J, u0) == 0
    eq2 = diff(J, u1) == 0
    eq3 = diff(J, u2) == 0
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Step 4: Solve the three equations for u0, u1, u2.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sol = solve([eq1, eq2, eq3], [u0, u1, u2]);
    
    % Convert symbolic solutions to numeric values.
    u0_opt = double(sol.u0);
    u1_opt = double(sol.u1);
    u2_opt = double(sol.u2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Step 5: Display the computed symbolic solution.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf('Time step %d, x = %.2f:\n', t, x);
    fprintf('   u0 = %.4f\n', u0_opt);
    fprintf('   u1 = %.4f\n', u1_opt);
    fprintf('   u2 = %.4f\n\n', u2_opt);

    %--------- Write your Code here ----------- 
    
    % In receding horizon control, only the _____ control input is applied.
    % u0_opt or  u1_opt or u2_opt?

    u_applied = ??;

    %---------------- End of Code ----------------
    
    % Update the state using the dynamics: x(k+1) = x(k) + u_applied.
    x = x + u_applied;
    
    % Save the trajectories for plotting.
    x_traj(t+1) = x;
    u_traj(t) = u_applied;
end

%% Visualization of the Closed-Loop Performance
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

% Plot control input trajectory
subplot(2,1,2);
stairs(0:Tsim-1, u_traj, 'k', 'LineWidth', 2);
xlabel('Time step'); ylabel('Control Input u');
title('Control Input Trajectory');
grid on;

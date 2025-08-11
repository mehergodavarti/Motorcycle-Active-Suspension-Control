clc
clear

% Define symbolic variables
syms z_wf dz_wf z_wr dz_wr z_b dz_b theta dtheta
syms u_f u_r

% Numerical values

    kf= 15000;
    kr= 17000;
    cf= 1000;
    cr= 1200;
    m_f= 4.8;
    m_r= 6.0;
    m_b= 180.0;
    Ib= 20.0;
    l_f= 1.0;
    l_r= 1.2;

% Define states and inputs
x = [z_wf; dz_wf; z_wr; dz_wr; z_b; dz_b; theta; dtheta];
u = [u_f; u_r];

% Relative deflections and velocities
delta_f  = z_b + l_f*theta - z_wf;
delta_r  = z_b - l_r*theta - z_wr;
ddelta_f = dz_b + l_f*dtheta - dz_wf;
ddelta_r = dz_b - l_r*dtheta - dz_wr;

% Equations of motion
ddz_wf = (kf*delta_f + cf*ddelta_f - u_f) / m_f;
ddz_wr = (kr*delta_r + cr*ddelta_r - u_r) / m_r;
ddz_b  = -(kf*delta_f + cf*ddelta_f + kr*delta_r + cr*ddelta_r) / m_b;
ddtheta = (-kf*l_f*delta_f - cf*l_f*ddelta_f + kr*l_r*delta_r + cr*l_r*ddelta_r) / Ib;

% First-order system
dx = [dz_wf;
      ddz_wf;
      dz_wr;
      ddz_wr;
      dz_b;
      ddz_b;
      dtheta;
      ddtheta];

%output state vector
y = [z_b;
    theta;
    dz_b;
    dtheta;
    ddz_b;
    ddtheta];

% Jacobians
A = double(jacobian(dx, x));
B = double(jacobian(dx, u));
C = double(jacobian(y,x));
D = double(jacobian(y,u));
% %C = [0 0 0 0 1 0 0 0;  % z_b
%          0 0 0 0 0 0 1 0]; % θ

% Compute eigenvalues
eig_A = eig(A);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define system matrices (A, B, C, D) here as before
% A = ...; B = ...; C = ...; D = ...;

% Augment B with disturbance input
B_disturbance = zeros(size(B,1),1);
B_disturbance(1) = 1;  % Assume disturbance affects front suspension dynamics
B = [B B_disturbance];

% Select output variables: z_b, z_b_dot, z_b_ddot, θ, θ_dot, θ_ddot
C_out = [0 0 0 0 1 0 0 0;     % z_b
         0 0 0 0 0 1 0 0;     % z_b_dot
         0 0 0 0 0 0 1 0;     % z_b_ddot (approximated as theta here)
         0 0 0 0 0 0 1 0;     % θ
         0 0 0 0 0 0 0 1;     % θ_dot
         0 0 0 0 0 0 0 0];    % θ_ddot placeholder
D_out = zeros(6, 3);  % B has 3 columns: 2 inputs + 1 disturbance

% Separate control input and disturbance input
B_u = B(:, 1:2);  % First two columns are control inputs
B_d = B(:, 3);    % Third column is disturbance input (e.g., road bump at front)

% Create augmented state-space model with disturbance input
plant = ss(A, [B_u B_d], C_out, D_out);

% Discretize model
Ts = 0.01;
plant_d = c2d(plant, Ts);

% Label inputs for MPC: control inputs as 'MV', disturbance as 'UD'
plant_d_mpc = setmpcsignals(plant_d, 'MV', 1:2, 'UD', 3);

% Create MPC object
mpc_obj = mpc(plant_d_mpc, Ts);

% Set horizons
mpc_obj.PredictionHorizon = 20;
mpc_obj.ControlHorizon = 5;

% Set weights
mpc_obj.Weights.OutputVariables = [1 1 0 1 0 1];  % Emphasize z_b, z_b_dot, θ
mpc_obj.Weights.ManipulatedVariables = [.01 .01];
mpc_obj.Weights.ManipulatedVariablesRate = [0.01 0.01];

% Set constraints (optional)
mpc_obj.MV(1).Min = -100;
mpc_obj.MV(1).Max = 100;
mpc_obj.MV(2).Min = -100;
mpc_obj.MV(2).Max = 100;

% Simulation time
T_sim = 20;
sim_steps = T_sim / Ts;

% Initial state
x0 = zeros(8,1);

% Reference trajectory: zeros for all outputs
r = zeros(sim_steps, 6);

% Define disturbance profile: front bump using Gaussian shape
bump_center = round(0.6 / Ts);
bump_width = round(0.1 / Ts);
t = (0:sim_steps-1)' * Ts;
d = 0.01 * exp(-((1:sim_steps)' - bump_center).^2 / (2 * bump_width^2));

% Define simulation options with disturbance
options = mpcsimopt(mpc_obj);
options.PlantInitialState = x0;
options.UnmeasuredDisturbance = d;

% Simulate closed-loop system with disturbance
[y, t, u, x] = sim(mpc_obj, sim_steps, r, options);

% Plot results
figure;
subplot(5,1,1);
plot(t, y(:,1), 'b');
title('MPC Control of Motorcycle Suspension');
ylabel('Bike z (m)');

subplot(5,1,2);
plot(t, y(:,2), 'r');
ylabel('z_b dot (m/s)');

subplot(5,1,3);
plot(t, y(:,4), 'k');
ylabel('Pitch \theta (rad)');

subplot(5,1,4);
plot(t, u(:,1), 'b'); hold on;
plot(t, u(:,2), 'r');
legend('Front Force','Rear Force');
ylabel('u_f, u_r (N)');

subplot(5,1,5);
plot(t, d, 'k');
ylabel('Disturbance (m)');
xlabel('Time (s)');
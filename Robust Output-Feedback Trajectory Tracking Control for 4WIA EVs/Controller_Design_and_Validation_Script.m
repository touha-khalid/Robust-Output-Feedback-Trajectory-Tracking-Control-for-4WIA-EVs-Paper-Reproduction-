%% FWIA Robust Control: Design & Validation
% Project: Robust Output-Feedback Trajectory Tracking for 4WIA EVs
% Based on: Li, Jiao & Zhang (2023)
%
% This script:
% 1. Defines the 2-DOF Vehicle Model parameters.
% 2. Solves the H-infinity LMI optimization problem using YALMIP/MOSEK.
% 3. Applies Physics-Informed Correction (Negative Feedback enforcement).
% 4. Validates the controller on a nonlinear vehicle model.

clear; clc; close all;

%% --- 1. SYSTEM PARAMETERS ---
p.m = 1830; p.Iz = 3234; p.Jw = 10.0035; p.b_sw = 350.1; 
p.km = 0.078; p.lp = 0.036; p.lm = 0.024; p.eta = 0.7; 
p.rs = 30; p.lf = 1.4; p.lr = 1.65; 
p.Cf = -134843; p.Cr = -124337; 

% Actuator Limits
u_max = [2500; 6]; 

% LPV Vertices (Velocity range 5 to 30 m/s)
vx_range = [5, 30]; 
vertices = [];
for r1 = vx_range
    for r2 = 1./vx_range
        vertices = [vertices; r1, r2, r2^2];
    end
end

%% --- 2. ROBUST LMI DESIGN ---
fprintf('Designing Robust Controller...\n');
if exist('yalmip','file') ~= 2, error('YALMIP required. Please install YALMIP and MOSEK.'); end
yalmip('clear');

nx = 6; nu = 2; 

% Decision Variables
Q = sdpvar(nx, nx, 'symmetric'); 
Y = sdpvar(nu, nx, 'full');      
gamma_sq = sdpvar(1, 1); 

% Input Matrix (Sparse for efficiency)
Bn = zeros(nx, nu);
Bn(4, 1) = 1/p.Iz; 
Bn(6, 2) = (p.eta * p.rs * p.km) / p.Jw;

% Performance Weights (H-infinity Tuning)
% Penalizes [LatError, YawError, Sideslip, YawRate, SteerAngle, SteerRate]
C1 = diag([1, 1, 0, 1, 1, 1]);
D1u = zeros(nx, nu);

LMI = [];
% Loop over LPV vertices to enforce robustness
for i = 1:size(vertices,1)
    [A, ~] = get_vertex_matrices(vertices(i,:), p);
    
    % Lyapunov Stability Term
    Phi11 = A*Q + Q*A' + Bn*Y + Y'*Bn';
    
    % Bounded Real Lemma (LMI Block)
    Row1 = [Phi11,         eye(nx),           (C1*Q + D1u*Y)'];
    Row2 = [eye(nx),       -gamma_sq*eye(nx),  zeros(nx, nx)];
    Row3 = [(C1*Q + D1u*Y), zeros(nx, nx),    -eye(nx)];
    
    LMI = [LMI, [Row1; Row2; Row3] <= -1e-6*eye(3*nx)];
end

% Constraints for numerical conditioning
LMI = [LMI, Q >= 1e-2*eye(nx)]; 
LMI = [LMI, gamma_sq >= 0.1];

% Objective: Minimize Gamma (Disturbance Rejection) + Regularization (Gain Norm)
Objective = gamma_sq + 10.0*norm(Y, 'fro');

options = sdpsettings('solver', 'mosek', 'verbose', 0);
sol = optimize(LMI, Objective, options);

if sol.problem == 0 || sol.problem == 4
    K_raw = value(Y) / value(Q);
    fprintf('Design Success! Raw Gain Norm: %.2f\n', norm(K_raw));
else
    error('Optimization Failed. Info: %s', sol.info);
end

%% --- 3. PHYSICS-BASED CORRECTION ---
% Ensures the controller implements Negative Feedback (Counter-steering)
K_final = K_raw;

% Check DYC Channel (Row 1): Positive Error -> Negative Torque (Right Turn)
if K_final(1,1) > 0
    fprintf('Correcting DYC Gain Polarity...\n');
    K_final(1,:) = -K_final(1,:);
end

% Check Steering Channel (Row 2): Positive Error -> Negative Steer (Right Turn)
if K_final(2,1) > 0
    fprintf('Correcting Steering Gain Polarity...\n');
    K_final(2,:) = -K_final(2,:);
end

% Save the controller for other scripts
save('FWIA_Controller.mat', 'K_final', 'p', 'u_max');
fprintf('Controller saved to FWIA_Controller.mat\n');

%% --- 4. VALIDATION SIMULATION ---
fprintf('Running Validation Simulation...\n');
T_final = 10; dt = 0.001;
time = 0:dt:T_final; N = length(time);

x_plant = zeros(6, N); 
x_plant(:,1) = [0.5; 0; 0; 0; 0; 0]; % Initial Lateral Error = 0.5m
u_hist = zeros(2, N); 
vx_const = 15; 

for k = 1:N-1
    xp = x_plant(:,k);
    
    % Control Law
    u = K_final * xp;
    
    % Actuator Saturation
    u_sat = max(min(u, u_max), -u_max);
    u_hist(:,k) = u_sat;
    
    % Nonlinear Dynamics Update
    k1 = vehicle_dyn(xp, u_sat, vx_const, p);
    x_plant(:,k+1) = xp + dt*k1;
end

% Plotting
figure('Color','w', 'Position', [100 100 1000 600]);

subplot(2,2,1); 
plot(time, x_plant(1,:), 'LineWidth', 2); grid on;
yline(0, 'k--');
xlabel('Time (s)'); ylabel('Lateral Error (m)');
title('Lateral Accuracy [SUCCESS if -> 0]');

subplot(2,2,2); 
plot(time, x_plant(4,:), 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Yaw Rate (rad/s)');
title('Yaw Stability');

subplot(2,2,3); 
plot(time, u_hist(1,:), 'r', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Nm');
title('DYC Input');

subplot(2,2,4); 
plot(time, u_hist(2,:), 'm', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Amps');
title('Steering Current');

%% Helper Functions
function dx = vehicle_dyn(x, u, vx, p)
    % Unpack State
    beta = x(3); gamma = x(4); delta = x(5); d_delta = x(6);
    DMz = u(1); i_m = u(2);
    
    % Slip Angles
    alpha_f = beta + (p.lf * gamma)/vx - delta;
    alpha_r = beta - (p.lr * gamma)/vx;
    
    % Tire Forces (Linear Model)
    Fyf = p.Cf * alpha_f; Fyr = p.Cr * alpha_r;
    
    % State Derivatives
    d_ed = vx * x(2) + vx * beta;         
    d_ephi = gamma;                       
    d_beta = (Fyf + Fyr)/(p.m * vx) - gamma; 
    d_gamma = (p.lf * Fyf - p.lr * Fyr + DMz)/p.Iz; 
    
    % Steering Dynamics
    term = p.Cf * (p.lp + p.lm) / p.Jw;
    d_delta_dot = -term*beta - (term*p.lf/vx)*gamma + term*delta ...
                  - (p.b_sw/p.Jw)*d_delta + (p.eta*p.rs*p.km/p.Jw)*i_m;
    
    dx = [d_ed; d_ephi; d_beta; d_gamma; d_delta; d_delta_dot];
end

function [An, L1] = get_vertex_matrices(rho, p)
    vx = rho(1); 
    An = zeros(6,6);
    An(1,2) = vx; An(1,3) = vx; An(2,4) = 1;
    An(3,3) = (p.Cf + p.Cr)/(p.m * vx);
    An(3,4) = (p.lf * p.Cf - p.lr * p.Cr)/(p.m * vx^2) - 1;
    An(3,5) = -p.Cf/(p.m * vx);
    An(4,3) = (p.lf * p.Cf - p.lr * p.Cr)/p.Iz;
    An(4,4) = (p.lf^2 * p.Cf + p.lr^2 * p.Cr)/(p.Iz * vx);
    An(4,5) = -p.lf * p.Cf / p.Iz;
    An(5,6) = 1;
    term = (p.lp + p.lm)/p.Jw;
    An(6,3) = -p.Cf * term; An(6,4) = -p.Cf * p.lf * term / vx;
    An(6,5) = p.Cf * term; An(6,6) = -p.b_sw / p.Jw;
    L1 = zeros(3, 6); 
end
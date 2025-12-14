%% FWIA Baseline: PID Controller Comparison
% Objective: Show that a simple PID fails under uncertainty where Robust H-inf succeeds.

clear; clc; close all;

p.m = 1830; p.Iz = 3234; p.Jw = 10.0035; p.b_sw = 350.1; 
p.km = 0.078; p.lp = 0.036; p.lm = 0.024; p.eta = 0.7; 
p.rs = 30; p.lf = 1.4; p.lr = 1.65; 
p.Cf = -134843; p.Cr = -124337; 
u_max = [2500; 6]; 

% PID Gains (Manually Tuned)
Kp_steer = -2.5; Kd_steer = -0.5;
Kp_dyc = -2000; Kd_dyc = -500;

T_final = 10; dt = 0.001; 
time = 0:dt:T_final; N = length(time);
x_plant = zeros(6, N); x_plant(:,1) = [0.5; 0; 0; 0; 0; 0]; 
vx_const = 15; 

for k = 1:N-1
    xp = x_plant(:,k);
    e_lat = xp(1); e_dot = xp(2); % Error states
    
    % Simple PID Law
    u_steer = Kp_steer * e_lat + Kd_steer * e_dot;
    u_dyc = Kp_dyc * e_lat + Kd_dyc * e_dot;
    
    u = [u_dyc; u_steer];
    u_sat = max(min(u, u_max), -u_max);
    
    k1 = vehicle_dyn(xp, u_sat, vx_const, p);
    x_plant(:,k+1) = xp + dt*k1;
end

plot(time, x_plant(1,:), 'LineWidth', 2); grid on;
title('PID Baseline Performance'); ylabel('Lateral Error (m)');

function dx = vehicle_dyn(x, u, vx, p)
    beta = x(3); gamma = x(4); delta = x(5); d_delta = x(6);
    DMz = u(1); i_m = u(2);
    alpha_f = beta + (p.lf * gamma)/vx - delta;
    alpha_r = beta - (p.lr * gamma)/vx;
    Fyf = p.Cf * alpha_f; Fyr = p.Cr * alpha_r;
    d_ed = vx * x(2) + vx * beta;         
    d_ephi = gamma;                       
    d_beta = (Fyf + Fyr)/(p.m * vx) - gamma; 
    d_gamma = (p.lf * Fyf - p.lr * Fyr + DMz)/p.Iz; 
    term = p.Cf * (p.lp + p.lm) / p.Jw;
    d_delta_dot = -term*beta - (term*p.lf/vx)*gamma + term*delta ...
                  - (p.b_sw/p.Jw)*d_delta + (p.eta*p.rs*p.km/p.Jw)*i_m;
    dx = [d_ed; d_ephi; d_beta; d_gamma; d_delta; d_delta_dot];
end
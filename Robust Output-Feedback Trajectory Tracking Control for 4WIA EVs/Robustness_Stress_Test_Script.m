%% FWIA Extension: Robustness Analysis & Stress Testing
% Objective: Evaluate controller performance under parameter uncertainties.
% Scenarios:
% 1. Nominal (Design Condition)
% 2. Heavy Load (+40% Mass)
% 3. Slippery Road (-60% Friction)

clear; clc; close all;

% 1. Load Controller
if exist('FWIA_Controller.mat', 'file')
    load('FWIA_Controller.mat');
else
    error('Controller file not found. Run FWIA_Design_Validation.m first.');
end

% 2. Define Scenarios
scenarios(1).name = 'Nominal Case';
scenarios(1).p = p;
scenarios(1).style = 'b-'; % Blue Solid

scenarios(2).name = 'Heavy Load (+40% Mass)';
scenarios(2).p = p;
scenarios(2).p.m = p.m * 1.40; 
scenarios(2).style = 'r--'; % Red Dashed

scenarios(3).name = 'Slippery Road (Wet/Ice)';
scenarios(3).p = p;
scenarios(3).p.Cf = p.Cf * 0.40; % 60% reduction
scenarios(3).p.Cr = p.Cr * 0.40;
scenarios(3).style = 'k-.'; % Black Dash-Dot

% 3. Simulation & Metrics Calculation
fprintf('\n--- ROBUSTNESS METRICS ---\n');
fprintf('| %-20s | %-12s | %-12s | %-15s |\n', 'Scenario', 'RMS Error (m)', 'Settling (s)', 'Max DYC (Nm)');
fprintf('|----------------------|--------------|--------------|-----------------|\n');

T_final = 10; dt = 0.001; 
time = 0:dt:T_final; N = length(time);
vx_const = 15; 

figure('Color','w', 'Position', [100 100 1200 600]);
sgtitle('Extension Results: Robustness under Uncertainty', 'FontSize', 14);

for s = 1:length(scenarios)
    p_curr = scenarios(s).p;
    x_plant = zeros(6, N); x_plant(:,1) = [0.5; 0; 0; 0; 0; 0]; 
    u_hist = zeros(2, N);
    
    for k = 1:N-1
        xp = x_plant(:,k);
        u = K_final * xp;
        u_sat = max(min(u, u_max), -u_max);
        u_hist(:,k) = u_sat;
        k1 = vehicle_dyn(xp, u_sat, vx_const, p_curr);
        x_plant(:,k+1) = xp + dt*k1;
    end
    
    % Metrics
    error_traj = x_plant(1,:);
    rms_err = sqrt(mean(error_traj.^2));
    idx_settle = find(abs(error_traj) > 0.01, 1, 'last');
    if isempty(idx_settle), t_settle = 0; else, t_settle = time(idx_settle); end
    max_dyc = max(abs(u_hist(1,:)));
    
    fprintf('| %-20s | %-12.4f | %-12.2f | %-15.0f |\n', ...
        scenarios(s).name, rms_err, t_settle, max_dyc);
    
    % Plotting
    subplot(2,2,1); hold on; plot(time, x_plant(1,:), scenarios(s).style, 'LineWidth', 1.5);
    subplot(2,2,2); hold on; plot(time, x_plant(4,:), scenarios(s).style, 'LineWidth', 1.5);
    subplot(2,2,3); hold on; plot(time, u_hist(1,:), scenarios(s).style, 'LineWidth', 1.5);
    subplot(2,2,4); hold on; plot(time, u_hist(2,:), scenarios(s).style, 'LineWidth', 1.5);
end

% Formatting
subplot(2,2,1); ylabel('Lateral Error (m)'); title('Tracking Accuracy'); grid on;
legend({scenarios.name}, 'Location', 'Best');
subplot(2,2,2); ylabel('Yaw Rate (rad/s)'); title('Stability'); grid on;
subplot(2,2,3); ylabel('DYC Torque (Nm)'); title('Control Effort'); grid on;
subplot(2,2,4); ylabel('Steering (A)'); title('Steering Input'); grid on;

%% Helper Function (Duplicate needed if file is standalone)
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
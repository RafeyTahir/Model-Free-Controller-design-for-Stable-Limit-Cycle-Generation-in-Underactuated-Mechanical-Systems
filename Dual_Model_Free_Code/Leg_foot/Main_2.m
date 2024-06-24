%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Leg-Foot Model System %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all

%%%% Physical Parameters
m_l = 1;  % mass of the leg
m_f = 0.2;  % mass of the foot
l = 1;    % length 1.2
I_f = (1/12) * m_f * l^2;  % Inertia of the Foot
I_l = (1/12) * m_l * l^2;  % Inertia of the Leg
k = 5000; % Stiffness
delta = 0.15; % Deformation parameter
g = 9.8;  % Gravitational acceleration

% Initial conditions
theta = 0;
alpha = 0;              % Actuated Joint angle
y = 0;
theta_dot = 0; alpha_dot = 0; y_dot = 0;
theta_ddot = 0; alpha_ddot = 0; y_ddot = 0;

% Control parameters
kp = 10; % Proportional gain
kd = 2; % Derivative gain
kp_1 = 2; % Secondary proportional gain
kd_1 = 6; % Secondary derivative gain
b1 = 2;
a = 50;

% Extended trajectory parameters
A = 0.175;
f = 1 / pi;
Ts = 0.001;
Ti = 0;
Tf = 30;

% Time vector
t = Ti:Ts:Tf-Ts; 
yyy = A * sin(2 * pi * f * t);
ref = yyy;
refdot = diff(yyy) / Ts;
refddot = diff(refdot) / Ts;

% Adjust lengths of reference signals
if length(refddot) < length(ref)
    refddot(end:length(ref)) = refddot(end);
end
if length(refdot) < length(ref)
    refdot(end:length(ref)) = refdot(end);
end

% Main simulation loop
for b = Ti+1:Tf
    for i = Ti+1:1000
        e = alpha - ref((b-1)*1000 + i);
        ed = alpha_dot - refdot((b-1)*1000 + i);
        edd = alpha_ddot - refddot((b-1)*1000 + i);
        alphaddot_des = refddot((b-1)*1000 + i);

        % System dynamics
        a1 = m_f + m_l;
        a2 = -l * m_l * sin(theta + alpha);
        a3 = m_l * l^2 + I_f + I_l;
        a4 = m_l * l^2 + I_l;
        a5 = -l * m_l * cos(theta + alpha) * (theta_dot + 2 * alpha_dot);
        a6 = -l * m_l * cos(theta + alpha) * alpha_dot;
        a7 = g * (m_f + m_l) + 2 * k * y;
        a8 = k * delta^2 * sin(2 * theta) - g * l * m_l * sin(theta + alpha);
        a9 = -g * l * m_l * sin(theta + alpha);

        Phi = alpha_ddot + (a2^2 * a8) / (a4 * (a4 * a1 - a2^2)) - (a2^2 * a4) / (a4 * (a1 * a4 - a2^2)) - a9 / a4 + a8 / a4;

        % Controller dynamics
        Phik1 = Phi;  
        Phidd = (Phik1 - Phik) / Ts;
        Phik_1 = Phik;
        Phik = Phik1;

        Phidk_1 = a * Xk_1 + Phi_hatd + b1 * ed;
        Phid = Phidk_1 + Phidk_1 - Phidk_2;
        Phidk_2 = Phidk_1;

        XX = edd + kd * ed + kp * e;
        Xk_1 = XX;
        Phi_hatd = -a * XX - b1 * ed + Phidk_1;
        Phi_hat = Ts * Phi_hatd + Phi_hat;
        U = Phi_hat + alphaddot_des - kd * ed - kp * e;

        Phi_hatstore(:, (b-1)*1000 + i) = Phi_hat + alpha_ddot;
        Phi_store(:, (b-1)*1000 + i) = Phi + alpha_ddot;
        XXstore(:, i) = XX;

        % State updates
        alpha_ddot = U / a4 - (a2^2 * a8) / (a4 * (a4 * a1 - a2^2)) + (a2^2 * a4) / (a4 * (a1 * a4 - a2^2)) + a9 / a4 + a8 / a4;
        theta_ddot = -a8 / a4 - alpha_ddot + (a2 * a7) / (a4 * a1 - a2^2) + (a2 * a6 * alpha_dot) / (a4 * a1 - a2^2) + (a2 * a5 * theta_dot) / (a4 * a1 - a2^2) + (a2^2 * a8) / (a4 * (a4 * a1 - a2^2));
        y_ddot = ((-a7 / a1 - (a6 * alpha_dot) / a1 - (a5 * theta_dot) / a1 + (a2 * a8) / (a1 * a4)) * (a4 * a1)) / (a4 * a1 - a2^2);

        ddot = [alpha_ddot; theta_ddot; y_ddot];
        dot = [alpha_dot; theta_dot; y_dot];
        ot = [alpha; theta; y];
        [ot, dot, ddot] = ODE1_integration(ot, dot, ddot, Ts);

        alpha = ot(1,:);
        alpha_dot = dot(1,:);
        alpha_ddot = ddot(1,:);
        theta = ot(2,:);
        theta_dot = dot(2,:);
        theta_ddot = ddot(2,:);
        y = ot(3,:);
        y_dot = dot(3,:);
        y_ddot = ddot(3,:);

        % Store data
        alpha_store(:, (b-1)*1000 + i) = ot(1,:);
        alphadot_store(:, (b-1)*1000 + i) = dot(1,:);
        alphaddot_store(:, (b-1)*1000 + i) = ddot(1,:);
        U_store(:, (b-1)*1000 + i) = U;
        traj(:, (b-1)*1000 + i) = ref((b-1)*1000 + i);
        error(:, (b-1)*1000 + i) = e;
        theta_store(:, (b-1)*1000 + i) = theta;
        thetad_store(:, (b-1)*1000 + i) = theta_dot;
        thetadd_store(:, (b-1)*1000 + i) = theta_ddot;
        y_store(:, (b-1)*1000 + i) = y;
        yd_store(:, (b-1)*1000 + i) = y_dot;
        ydd_store(:, (b-1)*1000 + i) = y_ddot;
    end
end

% Calculate RMSE
difference = traj - alpha_store;
squared_difference = difference .^ 2;
mean_squared_difference = mean(squared_difference);
rmse = sqrt(mean_squared_difference);
fprintf('The RMSE between the Desired and Estimated is: %f\n', rmse)
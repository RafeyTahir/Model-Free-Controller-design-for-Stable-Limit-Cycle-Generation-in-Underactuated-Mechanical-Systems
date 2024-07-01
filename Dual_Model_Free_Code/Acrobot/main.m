%%%%%%%%%%%%%%%%%%%%%%%% Acrobot-Example %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Clear workspace, close all figures, and clear command window
clc
clear all
close all

% Define system parameters
m1 = 1; % Mass of the first link
m2 = 1; % Mass of the second link
l1 = 1; % Length of the first link
l_c1 = 0.5; % Length to the center of mass of the first link
l_c2 = 0.5; % Length to the center of mass of the second link
I1 = 0.2; % Inertia of the first link
I2 = 1; % Inertia of the second link
g = 9.8; % Gravitational acceleration

% Initialize state variables
theta_1 = [0]; % Actuated joint angle
theta_2 = [0]; % Unactuated joint angle
theta_2dot = [0]; % Angular velocity of the unactuated joint
theta_2ddot = [0]; % Angular acceleration of the unactuated joint
theta_1dot = [0]; % Angular velocity of the actuated joint
theta_1ddot = [0]; % Angular acceleration of the actuated joint
int_e_o = [0]; % Integral of the error
extended_trajectory = [];

% Define control parameters
amplitude_factor = [0.08]; % Amplitude factor for reference trajectory
normalized_signal = [0];
p_min = [0];
p_max = [0];
G = [0];
GG = [0];
dist = 8; % Amplitude of Disturbance
e_2o = [0]; % Error in unactuated joint angle
e2o = [0];
e2do = [0]; % 1st derivative of the error.

% Controller gains
a_o = 2;
kd = 150; % Derivative gain
kp = 90; % Proportional gain
kp1 = 8; % Proportional gain for secondary controller
kd1 = 3; % Derivative gain for secondary controller
a = 100;
b1 = 2;
a1 = 50;

% Initialize matrices for control calculations
Phi_hat = [0]; Phi_hatdot = [0]; Phidk_2 = [0];
Xd = [0]; YY = [0]; Xk_1 = [0]; XX = [0]; U = [0];
Phi_hatk_1 = [0]; Phik_1 = [0]; Phik = Phik_1; Phi_hatk = Phi_hatk_1;
Phi_hatd = [0];

Phi_hat1 = [0]; Phi_hatdot1 = [0]; Phidk_21 = [0];
Xd1 = [0]; YY1 = [0]; Xk_11 = [0]; XX1 = [0]; U1 = [0];
Phi_hatk_11 = [0]; Phik_11 = [0]; Phik1 = Phik_11; Phi_hatk1 = Phi_hatk_11;
Phi_hatd1 = [0];

% Simulation parameters
Ts = 0.002; % Sampling time
Ts_2 = 0.01; % Secondary sampling time
Ti = 0; % Initial time
Tf = 15; % Final time
t = Ti:Ts_2:Tf-Ts_2; % Time vector

% Reference trajectory
f = 0.5; % Frequency
fs = 100; % Sampling frequency
amp = 0; % Amplitude of reference trajectory
ref_2 = amp * ones(size(t)); % Reference trajectory for actuated joint
ref_2dot = diff(ref_2) / Ts_2; % Derivative of the actuated joint reference trajectory
ref_2ddot = diff(ref_2dot) / Ts_2; % Second derivative of the reference trajectory

% Pad the reference trajectory derivatives to match the length of the original reference
if length(ref_2dot(1,:)) < length(ref_2)
    ref_2dot(1,length(ref_2dot(1,:)):length(ref_2)) = ref_2dot(1,length(ref_2dot(1,:)));
end
if length(ref_2ddot(1,:)) < length(ref_2)
    ref_2ddot(1,length(ref_2ddot(1,:)):length(ref_2)) = ref_2ddot(1,length(ref_2ddot(1,:)));
end

% Initialize trajectory storage
traj = zeros(1, 3000);

% Main simulation loop
for b = Ti+1:Tf
    % Generate reference trajectory for the current time step
    [ref1] = polynomial12(b, U1);    % Reference trajectory for underactuated joint generated through 6-D polynomial
    ref2 = ref1 * amplitude_factor;
    repeated_trajectory = flip(ref2);
    ref = [ref2, repeated_trajectory];    % Reference trajectory of underactuated joint 2-sec
    theta2_dot = diff(ref) / Ts;          % 1st derivative of Reference trajectory
    theta_ddot2 = diff(theta2_dot) / Ts;  % 2nd derivative of Reference trajectory

    % Pad the trajectory derivatives to match the length of the original reference
    if length(theta2_dot(1,:)) < length(ref)
        theta2_dot(1,length(theta2_dot(1,:)):length(ref)) = theta2_dot(1,length(theta2_dot(1,:)));
    end
    if length(theta_ddot2(1,:)) < length(ref)
        theta_ddot2(1,length(theta_ddot2(1,:)):length(ref)) = theta_ddot2(1,length(theta_ddot2(1,:)));
    end

    % Inner loop for control calculations
    for i = Ti+1:2000
        % Calculate errors and their integrals
        e_2 = theta_2 - ref(1,i);        % e_2 is the error of difference b\w actual position of unactuated joint and it's reference 
        int_e = e_2o + e_2 * Ts;
        e_2o = e_2;
        iint_e = int_e_o + int_e * Ts;
        int_e_o = int_e;
        ed_2 = theta_2dot - theta2_dot(:,i);     % 1st derivative of the error
        edd_2 = theta_2ddot - theta_ddot2(:,i);  % 2nd derivative of the error
        thetaddot_2 = theta_ddot2(:,i);

        % Calculate trigonometric functions of joint angles
        cc2 = cos(theta_2);
        ss2 = sin(theta_2);
        ss1 = sin(theta_1);
        s_12 = sin(theta_1 + theta_2);

        % Calculate system matrices
        m11 = m1 * (l_c1)^2 + m2 * ((l1)^2 + (l_c2)^2 + 2 * l1 * l_c2 * cc2) + I1 + I2;
        m12 = m2 * ((l_c2)^2 + l1 * l_c2 * cc2) + I2;
        m21 = m12;
        m22 = m2 * (l_c2)^2 + I2;
        h1 = -m2 * l1 * l_c2 * ss2 * (theta_2dot)^2 - 2 * m2 * l1 * l_c2 * ss2 * theta_2dot * theta_1dot;
        g1 = m2 * l_c2 * ss2 * (theta_1)^2;
        h2 = (m1 * l_c1 + m2 * l1) * g * ss1 + m2 * l_c2 * g * s_12;
        g2 = m2 * l_c2 * g * s_12;

        % Reduced system matrices
        m_2 = m11 - m21 * inv(m22) * m12;
        h_2 = h1 - m21 * inv(m22) * h2;
        g_2 = g1 - m21 * inv(m22) * g2;

        % Calculate the control input Phi
        Phi = (m_2) * theta_2ddot + h_2 + g_2 - theta_2ddot;    % Phi=F_{u} (As mention in paper) vector that contains all the system dynamics in terms of unactuated joint
        Phik1 = Phi; % k1=k+1
        Phidd = (Phik1 - Phik) / Ts;
        Phik_1 = Phik;
        Phik = Phik1;

        Phidk_1 = a * Xk_1 + Phi_hatd + b1 * ed_2;
        Phid = Phidk_1 + Phidk_1 - Phidk_2;
        Phidk_2 = Phidk_1;

        XX = edd_2 + kd * ed_2 + kp * e_2;
        Xk_1 = XX;

        Phi_hatd = -a * XX - b1 * ed_2 + Phidk_1;
        Phi_hat = Ts * Phi_hatd + Phi_hat;              % Estimate of Phi or F_{u} vector

        U2 = (Phi_hat + thetaddot_2 - kd * ed_2 - kp * e_2) * 1 / (-m21 * inv(m22));  
        Phi_hatstore(:,i) = Phi_hat + theta_2ddot;
        Phi_store(:,i) = Phi + theta_2ddot;
        U = U2 * (-m21 * inv(m22)) + theta_2ddot;        % Control Input (Low Level Controller)
        U_store(:,i) = U;

        % Store the computed value of XX
        XXstore(:,i) = XX;

        % Update the state variables for the next iteration
        theta2dnew = theta_2dot + Ts * (U - (m_2) * theta_2ddot - h_2 - g_2); % Compute new angular velocity for joint 2
        theta2ddnew = (theta2dnew - theta_2dot) / Ts; % Compute new angular acceleration for joint 2
        theta2new = theta2dnew * Ts + theta_2; % Compute new angle for joint 2

        % Update state variables
        theta_2ddot = theta2ddnew;
        theta_2dot = theta2dnew;
        theta_2 = theta2new;

        % Store the results
        theta2_store(:, (b-1) * 2000 + i) = theta_2;
        theta2d_store(:, (b-1) * 2000 + i) = theta_2dot;
        theta2dd_store(:, (b-1) * 2000 + i) = theta_2ddot;
        U_store(:, (b-1) * 2000 + i) = U;
        traj(:, (b-1) * 2000 + i) = ref(:, i);
        trajdot(:, (b-1) * 2000 + i) = theta2_dot(:, i);
        Norm_store(:, (b-1) * 2000 + i) = U1;
        error(:, (b-1) * 2000 + i) = e_2;

        % Update matrices for the actuated joint (joint 1)
        m_1 = m22 - m12 * inv(m11) * m21;
        h_1 = -m12 * inv(m11) * h1 + h2;
        g_1 = -m12 * inv(m11) * g1 + g2;

        % Compute the new state variables for the actuated joint (joint 1)
        theta_11ddot = -h_1 * inv(m_1) - g_1 * inv(m_1) + U * inv(m_1);
        theta1dnew = theta_1dot + Ts * theta_11ddot;
        theta1ddnew = (theta1dnew - theta_1dot) / Ts;
        theta1new = theta1dnew * Ts + theta_1;

        % Update state variables
        theta_1ddot = theta1ddnew;
        theta_1dot = theta1dnew;
        theta_1 = theta1new;

        % Store the results
        theta1_store(:, (b-1) * 2000 + i) = theta_1;
        theta1d_store(:, (b-1) * 2000 + i) = theta_1dot;
        theta1dd_store(:, (b-1) * 2000 + i) = theta_1ddot;

        % Additional control calculations for adaptive control (e2, ed2, e2dd)  after half-period
        if i == 1000
            avg = [theta1_store(:, ((b*1000) + 1) - i), theta_1];
            A2 = mean(avg);
            e2 = A2 - ref_2(:,1);
            error2(:, b) = e2;
            ed2 = (e2 - e2o) / Ts;
            e2o = e2;
            e2dd = (ed2 - e2do) / Ts;
            ed2o = ed2;

            % Compute Phi=F_{a} and its derivatives for joint 1  
            Phi1 = (m_1) * theta_1ddot + h_1 + g_1 - theta_1ddot;
            Phik11 = Phi1;
            Phidd1 = (Phik11 - Phik1) / Ts_2;
            Phik_11 = Phik1;
            Phik1 = Phik11;

            % Update the control input for joint 1 
            Phidk_11 = a1 * Xk_11 + Phi_hatd1 + b1 * ed_2;
            Phid1 = Phidk_11 + Phidk_11 - Phidk_21;
            Phidk_21 = Phidk_11;
            XX1 = edd_2 + kd1 * ed_2 + kp1 * e_2;
            Xk_11 = XX1;
            Phi_hatd1 = -a1 * XX1 - b1 * ed_2 + Phidk_11;
            Phi_hat1 = Ts * Phi_hatd1 + Phi_hat1;
            U4 = (Phi_hat1 - kd1 * ed_2 - kp1 * e_2); % Control Input (High Level Controller)
        end
    end

        % Adjust the control parameter U4 based on the error (Actuated Joint) e2 (Normalization)
  if e2 < -0.5
            p = 0.38; % Large negative error
        elseif e2 < -0.05 && e2 > -0.5
            input_range = [-0.5, -0.05];
            output_range = [0.42, 0.49];
            p = interp1(input_range, output_range, e2, 'linear');
        elseif e2 > 0.5
            p = 0.62; % Large positive error
        elseif e2 > 0.05 && e2 < 0.5
            input_range = [0.05, 0.5];
            output_range = [0.51, 0.58];
            p = interp1(input_range, output_range, e2, 'linear');
        elseif e2 < 0.05 && e2 > -0.07
            p = 0.5;
        end
        U1 = p;   % Final Parameter that will be used by polynomail for trajectory generation.

    end % End of the main loop

    % Calculate the RMSE between the desired and estimated trajectories
    difference = traj - theta2_store;
    squared_difference = difference .^ 2;
    mean_squared_difference = mean(squared_difference);
    rmse = sqrt(mean_squared_difference);
    fprintf('The RMSE between the Desired and Estimated is: %f\n', rmse);

    % Plot the results
    % figure;
    % plot(ref_2 + traj, '--b');
    % hold on;
    % plot(theta1_store + theta2_store, 'g');
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % legend({'$\theta^{*}_{ua}+\theta^{*}_{a}$', '$\hat{\theta}_{ua}+\hat{\theta}_{a}$'}, 'interpreter', 'latex', 'Location', 'best');
    % xlabel('$time(s)$', 'interpreter', 'latex');
    % figure(gcf);
    % 
    % figure;
    % plot(ref_2dot + theta2d_store, '--b');
    % hold on;
    % plot(theta1d_store + theta2d_store, 'r');
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % legend({'$\dot{\theta}^{*}_{ua}+\dot{\theta}^{*}_{a}$', '$\hat{\dot{\theta}}_{ua}+\hat{\dot{\theta}}_{a}$'}, 'interpreter', 'latex', 'Location', 'best');
    % xlabel('$time(s)$', 'interpreter', 'latex');
    % 
    % figure;
    % plot(theta1_store);
    % hold on;
    % title(['Actuated Joint Position']);
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % xlabel('$time(s)$', 'interpreter', 'latex');
    % 
    % figure;
    % plot(Norm_store);
    % ylim(gca, [-1, 1]);
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % xlabel('$time(s)$', 'interpreter', 'latex');
    % 
    % figure;
    % plot3(theta1_store + theta2_store, theta1d_store + theta2d_store, t);
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % xlabel('$(\theta_{ua}+\theta_{a})$', 'interpreter', 'latex');
    % ylabel('$(\dot{\theta}_{ua}+\dot{\theta}_{a})$', 'interpreter', 'latex');
    % 
    % figure;
    % plot(U_store);
    % set(gca, 'XTick', [0 500 1000 1500 2000 2500 3000]);
    % set(gca, 'XTickLabel', [0 5 10 15 20 25 30]);
    % xlabel('$time(s)$', 'interpreter', 'latex');

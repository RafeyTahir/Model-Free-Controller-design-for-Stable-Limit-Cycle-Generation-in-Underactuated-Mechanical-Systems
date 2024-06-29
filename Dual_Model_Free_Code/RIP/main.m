%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rotary Inverted Pendulum %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;

%%%% Physical Parameters
mr = 0.095;  % Mass of the rotary arm
mp = 0.024;  % Mass of the pendulum
lr = 0.085;  % Length of the rotary arm
lp = 0.129;  % Length of the pendulum
l1 = 0.0645; % Distance to the pendulum's center of mass
jp = 3.3e-5; % Pendulum inertia
jr = 5.7e-5; % Rotary arm inertia
g = 9.8;     % Acceleration due to gravity

% Initial Conditions
theta_1 = 0;        % Actuated Joint angle
theta_2 = 0;        % Unactuated Joint angle
theta_2dot = 0;     % Unactuated Joint angular velocity
theta_2ddot = 0;    % Unactuated Joint angular acceleration
theta_1dot = 0;     % Actuated Joint angular velocity
theta_1ddot = 0;    % Actuated Joint angular acceleration
int_e_o = 0;        % Initial integrated error
e_2o = 0;           % Initial error
e2do = 0;           % Initial error derivative

% Trajectory Initialization
theta_1dots = zeros(1, 30000);
theta_1s = zeros(1, 30000);
extended_trajectory = [];
amplitude_factor = [-5, 5];
aaa = 0.087;
normalized_signal = 0;
p_min = 0;
p_max = 0;

% Controller Parameters
kd = 2;        % Derivative gain
kp = 0.5;      % Proportional gain
kp_1 = 2;      % Proportional gain for second controller
kd_1 = 6;      % Derivative gain for second controller
b1 = 2;        % Controller parameter
a = 50;        % Controller parameter

% Preallocate Variables
Phi_hat = 0; Phi_hatdot = 0; Phidk_2 = 0;
Xd = 0; YY = 0; Xk_1 = 0; XX = 0; U = 0;
Phi_hatk_1 = 0; Phik_1 = 0; Phik = Phik_1; Phi_hatk = Phi_hatk_1;
Phi_hatd = 0;

Phi_hat1 = 0; Phi_hatdot1 = 0; Phidk_21 = 0;
Xd1 = 0; YY1 = 0; Xk_11 = 0; XX1 = 0; U1 = 0;
Phi_hatk_11 = 0; Phik_11 = 0; Phik1 = Phik_11; Phi_hatk1 = Phi_hatk_11;
Phi_hatd1 = 0;

% Time Variables
Ts = 0.002;   % Sampling time
Ts_2 = 0.002;
Ti = 0;
Tf = 15;      % Final time
t = Ti:Ts_2:Tf-Ts_2;  % Time vector
amp = 0;      % Amplitude of the reference signal for underactuated Joint
ref_2 = amp * ones(size(t));  % Reference signal for underactuated Joint

% Calculate derivatives of the reference signal
ref_2dot = diff(ref_2) / Ts_2;
ref_2ddot = diff(ref_2dot) / Ts_2;

% Adjust lengths of reference signal derivatives
if length(ref_2dot) < length(ref_2)
    ref_2dot = [ref_2dot, ref_2dot(end)];
end

if length(ref_2ddot) < length(ref_2)
    ref_2ddot = [ref_2ddot, ref_2ddot(end)];
end

% Preallocate Trajectory
traj = zeros(1, 3000);

% Main Loop for Control
for b = Ti+1:Tf
    [ref1] = polynomial12(b, U1); % Reference Signal generated through 6-Degree polynomial
    ref2 = ref1 * aaa;
    repeated_trajectory = flip(ref2);
    ref = [ref2, repeated_trajectory];

    theta2_dot = diff(ref) / Ts;
    theta_ddot2 = diff(theta2_dot) / Ts;

    % Adjust lengths of calculated derivatives
    if length(theta2_dot) < length(ref)
        theta2_dot = [theta2_dot, theta2_dot(end)];
    end

    if length(theta_ddot2) < length(ref)
        theta_ddot2 = [theta_ddot2, theta_ddot2(end)];
    end

    for i = Ti+1:2000
        e_2 = theta_2 - ref(i);
        int_e = e_2o + e_2 * Ts;
        e_2o = e_2;
        iint_e = int_e_o + int_e * Ts;
        int_e_o = int_e;
        ed_2 = theta_2dot - theta2_dot(i);
        edd_2 = theta_2ddot - theta_ddot2(i);
        thetaddot_2 = theta_ddot2(i);
        cc = cos(theta_2);
        ss = sin(theta_2);
        ss_2 = (1 - cos(2*theta_2)) / 2;
        A = jr + mp * lr^2 + (mp * (lp^2) * ss_2) / 4;
        B = (mp * lr * lp * cc) / 2;
        C = ((mp * lp^2) / 4) + jp;
        D = mp * g * (lp/2) * ss - (mp/2) * lr * ss * theta_1dot * theta_2dot;
        Phi = (C - (B^2) / A) * theta_2ddot + D - theta_2ddot;
        Phik1 = Phi;  % k1 = k + 1
        Phidd = (Phik1 - Phik) / Ts_2;
        Phik_1 = Phik;
        Phik = Phik1;

        Phidk_1 = a * Xk_1 + Phi_hatd + b1 * ed_2;
        Phid = Phidk_1 + Phidk_1 - Phidk_2;
        Phidk_2 = Phidk_1;
        XX = edd_2 + kd * ed_2 + kp * e_2;
        Xk_1 = XX;

        Phi_hatd = -a * XX - b1 * ed_2 + Phidk_1;
        Phi_hat = Ts_2 * Phi_hatd + Phi_hat;

        Phi_hat11 = -a * (ed_2 + kd * e_2 + kp * int_e);  % m = 1
        Phi_hat22 = -a * (ed_2 + kd * e_2 + kp * int_e) - a_o * (e_2 + kd * int_e + iint_e * kp);

        U = (Phi_hat22 + thetaddot_2 - kd * ed_2 - kp * e_2);  % Control input

        % Store Variables
        Phi_hatstore(:, (b-1)*2000 + i) = Phi_hat11 + theta_2ddot;
        Phi_hatstore1(:, (b-1)*2000 + i) = Phi_hat22 + theta_2ddot;
        Phi_store(:, (b-1)*2000 + i) = Phi + theta_2ddot;
        U = U;
        XXstore(:, i) = XX;

        % Update Angular Accelerations
        theta_2ddot = (U - D) * A / (C - B^2);
        theta2dnew = theta_2dot + Ts * theta_2ddot;
        theta2ddnew = (theta2dnew - theta_2dot) / Ts;
        theta2new = theta2dnew * Ts + theta_2;

        % Update Angles and Velocities
        theta_2ddot = theta2ddnew;
        theta_2dot = theta2dnew;
        theta_2 = theta2new;

        % Store Data for Analysis
        theta2_store(:, (b-1)*2000 + i) = theta_2;
        theta2d_store(:, (b-1)*2000 + i) = theta_2dot;
        theta2dd_store(:, (b-1)*2000 + i) = theta_2ddot;
        U_store(:, (b-1)*2000 + i) = U;
        traj(:, (b-1)*2000 + i) = ref(i);
        trajdot(:, (b-1)*2000 + i) = theta2_dot(i);
        Norm_store(:, (b-1)*2000 + i) = U1;
        error(:, (b-1)*2000 + i) = e_2;

        % Update Actuated Joint Parameters
        theta_11ddot = (U + B * D / C) / ((A - B^2) / C);
        theta1dnew = theta_1dot + Ts * theta_11ddot;
        theta1ddnew = (theta1dnew - theta_1dot) / Ts;
theta1new = theta1dnew * Ts + theta_1;

% Update Angles and Velocities
theta_1ddot = theta1ddnew;
theta_1dot = theta1dnew;
theta_1 = theta1new;

% Store Data for Analysis
theta1_store(:, (b-1)*2000 + i) = theta_1;
theta1d_store(:, (b-1)*2000 + i) = theta_1dot;
theta1dd_store(:, (b-1)*2000 + i) = theta_1ddot;

% Intermediate calculations for every 1000th iteration
if i == 1000
    avg = [theta1_store(:, ((b*1000)+1)-i), theta_1];
    A2 = mean(avg);
    e2 = A2 - ref_2(:, 1);
    error2(:, b) = e2;
    ed2 = (e2 - e2o) / Ts;
    e2o = e2;
    e2dd = (ed2 - e2do) / Ts;
    e2do = ed2;

    % Update for the second controller
    Phi1 = theta_11ddot * ((A - B^2) / C) - B * D / C - theta_11ddot;
    Phik11 = Phi1;
    Phidd1 = (Phik11 - Phik1) / Ts_2;
    Phik_11 = Phik1;
    Phik1 = Phik11;

    Phidk_11 = a * Xk_11 + Phi_hatd1 + b1 * ed2;
    Phid1 = Phidk_11 + Phidk_11 - Phidk_21;
    Phidk_21 = Phidk_11;

    XX1 = e2dd + kd_1 * ed2 + kp_1 * e2;
    Xk_11 = XX1;

    Phi_hatd1 = -a * XX1 - b1 * e2 + Phidk_11;
    Phi_hat1 = Ts * Phi_hatd1 + Phi_hat1;

    U4 = (Phi_hat1 - kd_1 * ed2 - kp_1 * e2);
end
end

% Adjust control input U1 based on the error
if e2 < -0.5
    p = 0.38;  % Large negative error
elseif e2 < -0.05 && e2 > -0.5
    input_range = [-0.5, -0.05];
    output_range = [0.42, 0.49];
    p = interp1(input_range, output_range, e2, 'linear');
elseif e2 > 0.5
    p = 0.62;  % Large positive error
elseif e2 > 0.05 && e2 < 0.5
    input_range = [0.05, 0.5];
    output_range = [0.51, 0.58];
    p = interp1(input_range, output_range, e2, 'linear');
elseif e2 < 0.05 && e2 > -0.07
    p = 0.5;
end
U1 = p;

end

% Calculate the Root Mean Square Error (RMSE)
difference = traj - theta2_store;
squared_difference = difference .^ 2;
mean_squared_difference = mean(squared_difference);
rmse = sqrt(mean_squared_difference);
fprintf('The RMSE between the Desired and Estimated is: %f\n', rmse);

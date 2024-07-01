# Rotary Inverted Pendulum Control System

This MATLAB script simulates the control of a rotary inverted pendulum system using a proportional-derivative (PD) controller. The main objective is to stabilize the pendulum in the upright position and control its trajectory.

## Table of Contents
- [Introduction](#introduction)
- [Physical Parameters](#physical-parameters)
- [Initial Conditions](#initial-conditions)
- [Trajectory Initialization](#trajectory-initialization)
- [Controller Parameters](#controller-parameters)
- [Time Variables](#time-variables)
- [Main Loop for Control](#main-loop-for-control)
- [Root Mean Square Error Calculation](#root-mean-square-error-calculation)
- [How to Run](#how-to-run)

## Introduction

The rotary inverted pendulum is a classic problem in control theory and robotics. It consists of a pendulum attached to a rotary arm, and the goal is to control the arm's rotation to keep the pendulum upright. This script uses a PD controller to achieve this goal by calculating the necessary control inputs to maintain the pendulum's balance and follow a desired trajectory.

## Physical Parameters

These are the physical parameters of the rotary inverted pendulum system:

```matlab
mr = 0.095;  % Mass of the rotary arm
mp = 0.024;  % Mass of the pendulum
lr = 0.085;  % Length of the rotary arm
lp = 0.129;  % Length of the pendulum
l1 = 0.0645; % Distance to the pendulum's center of mass
jp = 3.3e-5; % Pendulum inertia
jr = 5.7e-5; % Rotary arm inertia
g = 9.8;     % Acceleration due to gravity
```

## Initial Conditions

The initial conditions for the system are set as follows:

```matlab
theta_1 = 0;        % Actuated Joint angle
theta_2 = 0;        % Unactuated Joint angle
theta_2dot = 0;     % Unactuated Joint angular velocity
theta_2ddot = 0;    % Unactuated Joint angular acceleration
theta_1dot = 0;     % Actuated Joint angular velocity
theta_1ddot = 0;    % Actuated Joint angular acceleration
int_e_o = 0;        % Initial integrated error
e_2o = 0;           % Initial error
e2do = 0;           % Initial error derivative
```

## Trajectory Initialization

```matlab
theta_1dots = zeros(1, 30000);
theta_1s = zeros(1, 30000);
extended_trajectory = [];
amplitude_factor = [-5, 5];
aaa = 0.087;       % Amplitude in radians
normalized_signal = 0;
p_min = 0;
p_max = 0;
```

## Controller Parameters

```matlab
kd = 2;        % Derivative gain
kp = 0.5;      % Proportional gain
kp_1 = 2;      % Proportional gain for second controller
kd_1 = 6;      % Derivative gain for second controller
b1 = 2;        % Controller parameter
a = 50;        % Controller parameter
```

## Time Variables

```matlab
Ts = 0.002;   % Sampling time
Ts_2 = 0.002;
Ti = 0;
Tf = 15;      % Final time
t = Ti:Ts_2:Tf-Ts_2;  % Time vector
amp = 0;      % Amplitude of the reference signal for actuated Joint
ref_2 = amp * ones(size(t));  % Reference signal for actuated Joint
```

## Main Loop for Control

The main loop runs the control algorithm, updating the state of the system at each time step. It generates the reference trajectory, calculates control inputs, and updates the system states accordingly.

```matlab
for b = Ti+1:Tf
    % Reference signal generation
    % Trajectory and error calculations
    % Control input calculations
    % State updates and data storage
end
```

## Root Mean Square Error Calculation

At the end of the simulation, the script calculates the RMSE between the desired and estimated trajectories.

```matlab
difference = traj - theta2_store;
squared_difference = difference .^ 2;
mean_squared_difference = mean(squared_difference);
rmse = sqrt(mean_squared_difference);
fprintf('The RMSE between the Desired and Estimated is: %f\n', rmse);
```

## How to Run

1. Ensure you have MATLAB installed.
2. Copy the script into a new MATLAB file (e.g., `rotary_inverted_pendulum.m`).
3. Run the script in MATLAB.

This script will simulate the rotary inverted pendulum control and print the RMSE at the end of the simulation.

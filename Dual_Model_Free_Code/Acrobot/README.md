# Acrobot Simulation

This MATLAB script simulates the control of an Acrobot system. The Acrobot is a two-link pendulum with an actuator at the first joint. This example demonstrates the use of adaptive control to manage the system dynamics.

## System Parameters
The following parameters define the physical properties of the Acrobot:

- `m1`: Mass of the first link (kg)
- `m2`: Mass of the second link (kg)
- `l1`: Length of the first link (m)
- `l_c1`: Length to the center of mass of the first link (m)
- `l_c2`: Length to the center of mass of the second link (m)
- `I1`: Inertia of the first link (kg.m^2)
- `I2`: Inertia of the second link (kg.m^2)
- `g`: Gravitational acceleration (m/s^2)

## State Variables
The state of the system is described by:

- `theta_1`: Actuated joint angle (rad)
- `theta_2`: Unactuated joint angle (rad)
- `theta_2dot`: Angular velocity of the unactuated joint (rad/s)
- `theta_2ddot`: Angular acceleration of the unactuated joint (rad/s^2)
- `theta_1dot`: Angular velocity of the actuated joint (rad/s)
- `theta_1ddot`: Angular acceleration of the actuated joint (rad/s^2)
- `int_e_o`: Integral of the error (for control purposes)

## Control Parameters
The script uses several control parameters to generate reference trajectories and control signals:

- `amplitude_factor`: Amplitude factor for the reference trajectory
- `dist`: Amplitude of disturbance
- `a_o`, `kd`, `kp`, `kp1`, `kd1`, `a`, `b1`, `a1`: Controller gains

## Simulation Parameters
- `Ts`: Sampling time (s)
- `Ts_2`: Secondary sampling time (s)
- `Ti`: Initial time (s)
- `Tf`: Final time (s)
- `t`: Time vector

## Reference Trajectory
A reference trajectory is generated for the actuated joint:

- `f`: Frequency of the trajectory
- `fs`: Sampling frequency
- `amp`: Amplitude of the reference trajectory
- `ref_2`, `ref_2dot`, `ref_2ddot`: Reference trajectory and its derivatives

## Main Simulation Loop
The simulation loop iterates over the specified time range, generating reference trajectories and computing control inputs for each time step. The control inputs are designed to minimize the error between the actual and desired trajectories.

### Error Calculation
The error (`e_2`) is computed as the difference between the actual position of the unactuated joint and its reference. This error, along with its derivatives, is used to update the control input.

### System Dynamics
The script calculates the system matrices and uses them to compute the control input (`Phi`). This input is then used to update the state variables for the next iteration.

### Adaptive Control
The script includes an adaptive control mechanism to adjust the control parameters based on the error. This helps improve the performance of the controller over time.

## Results and Visualization
After the simulation, the script calculates the root mean square error (RMSE) between the desired and estimated trajectories and prints the result. The script also includes commented-out code for plotting various aspects of the simulation results, such as:

- Reference and actual trajectories
- Angular velocities and accelerations
- Control inputs
- Errors

To visualize the results, uncomment the plotting sections in the script.
<div align="center">
<img width="605" src="Dual_Model_Free_Code/Acrobot/Tracking of unactuated joint.png" />
</div>
<div align="center">
<img width="605" src="Dual_Model_Free_Code/Acrobot/Velocity of underactuated joint.png" />
</div>
<div align="center">
<img width="605" src="Dual_Model_Free_Code/Acrobot/Control input.png" />
</div>

## Usage
Run the script in MATLAB to simulate the Acrobot control system. Adjust the parameters as needed to explore different control scenarios and observe the system's behavior.


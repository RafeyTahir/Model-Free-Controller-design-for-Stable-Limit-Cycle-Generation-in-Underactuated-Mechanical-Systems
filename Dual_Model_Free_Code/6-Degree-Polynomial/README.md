# Reference Trajectory Generation using 6th-degree Polynomial

This repository contains MATLAB code for generating reference trajectories using a 6th-degree polynomial function. The code calculates and plots reference trajectories for various values of the parameter `p`.

## Description

The goal of this project is to generate stable limit cycles for the non-actuated coordinate \(q_{na}\) using parameterized polynomial functions. These trajectories must be twice continuously differentiable and exhibit periodic behavior. The generated reference trajectories are used to bring the active joint \(q^{*}_{a}\) to a desired state.

## Code Explanation

The main script performs the following steps:

1. **Define the Polynomial Function**: A function handle `f` is defined to represent the 6th-degree polynomial.
2. **Set Time Interval and Step Size**: The time interval `[t_min, t_max]` and step size `h` are defined.
3. **Initialize Time and Reference Trajectory Arrays**: Arrays `t` and `ref` are initialized.
4. **Set Polynomial Coefficients**: The polynomial coefficients are calculated for different values of `p`.
5. **Generate and Plot Reference Trajectories**: For each value of `p`, the reference trajectory is generated and plotted.

### MATLAB Code

```matlab
clc
clear all

% Define the polynomial function
f = @(t, a_6, a_5, a_4, a_3, a_2, a_1, a_0) a_6*t^6 + a_5*t^5 + a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;

% Set the time interval and step size
t_min = 0;
t_max = 1;
h = 0.01;

% Initialize the time and reference trajectory arrays
t = t_min:h:t_max;
ref = zeros(size(t));

% Set the polynomial coefficients for different values of 'p'
p_values = [0.5, 0.65, 0.58, 0.42, 0.35];

for p_idx = 1:length(p_values)
    p = p_values(p_idx);
    
    % Calculate the polynomial coefficients
    a_6 = (2*p-1)*(6*p^4 - 12*p^3 + 4*p^2 + 2*p + 1)/(p^3 * (p - 1)^3);
    a_5 = -3*(4*p^6 - 18*p^4 + 16*p^3 - 1)/(p^3 * (p - 1)^3);
    a_4 = 3*(10*p^6 - 18*p^5 + 10*p^3 - 1)/(p^3 * (p - 1)^3);
    a_3 = -(20*p^6 - 48*p^5 + 30*p^4 - 1)/(p^3 * (p - 1)^3);
    
    % Generate the reference trajectory
    for i = 1:length(t)
        ref(i) = f(t(i), a_6, a_5, a_4, a_3, 0, 0, 1);
    end
    
    % Plot the reference trajectory
    hold on
    plot(t, ref, '--', 'DisplayName', ['p = ' num2str(p)]);
end

xlabel('Time (s)')
ylabel('Amplitude')
legend('Location', 'best')
title('Reference Trajectories for Different p Values')
grid on
print('-dpng', '-r300', 'poly');  % Save the plot as a PNG file
```

### Explanation

</div>
<div align="center">
<img width="605" src="Dual_Model_Free_Code/Results/poly.png" />
</div>

To generate a stable limit cycle, reference trajectories \(q^{*}_{na}(b,\Gamma,t)\) for the non-actuated coordinate are designed. These trajectories must belong to \(C^{2}\) to ensure no discontinuities in position, velocity, and acceleration. The high-level controller calculates the value of the parameter `b` to bring the active joint \(q^{*}_{a}\) to a desired state. 

The boundary conditions for each half period are:
\[
\begin{cases}
    q_{na}^{*}(b,\Gamma,0) = q_{na}^{*}(b,\Gamma,\Gamma)=A \\
    q_{na}^{*}(b,\Gamma,\frac{\Gamma}{2})=-A \\
    \Dot{q}_{na}^{*}(b,\Gamma,0)=\Dot{q}_{na}^{*}(b,\Gamma,\frac{\Gamma}{2})=\Dot{q}_{na}^{*}(b,\Gamma,\Gamma)=0.
\end{cases}
\]

By choosing a 6th-degree polynomial function parameterized with `b`, we can define \(\mathcal{B}(t,b)\) as:
\[
\mathcal{B}(t,b) = \sum^{6}_{i=0} \alpha_{i}(b)t^{i}
\]

The coefficients \(\alpha_{i}(b)\) are calculated as:
\[
\begin{align}
    \alpha_{6}(b) = \frac{(2b -1)(6b^4 -12b^3 + 4b^{2} + 2b +1)}{b^{3}(b-1)^{3}},\nonumber\\
    \alpha_{5}(b) =\frac{-3(4b^{6} -18b^4 + 16b^3 -1)}{b^3(b-1)^3},\nonumber\\
    \alpha_{4}(b) = \frac{3(10b^6 -18b^5 + 10b^3 -1)}{b^{3}(b-1)^{3}},\nonumber\\
    \alpha_{3}(b) = \frac{-(20b^{6} -48b^{5} + 30b^{4} -1)}{b^{3}(b-1)^{3}},\nonumber\\
    \alpha_{2}(b) = 0, \hspace{0.35cm}
    \alpha_{1}(b) = 0,\hspace{0.3cm}
    \alpha_{0}(b) = 1.
\end{align}
\]

For \(b = 0.5\), the trajectory spends equal amounts of time on both sides of the point of unstable equilibrium.

### References

- Andary, S., et al. "Control Design for Underactuated Mechanical Systems." (2009).
- Andary, S., et al. "Dual Controller Design for Underactuated Mechanical Systems." (2012).

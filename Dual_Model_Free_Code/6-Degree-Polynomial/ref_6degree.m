% Reference Trajectory Generation using 6th-degree polynomial

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

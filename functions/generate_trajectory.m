function q_traj = generate_trajectory(q_start, q_end, num_steps)
% GENERATE_TRAJECTORY - Generates a smooth joint-space trajectory.
%   Uses a quintic polynomial to ensure zero velocity and acceleration
%   at the start and end points for smooth motion.
%
%   Inputs:
%       q_start   - 1xN array of starting joint angles
%       q_end     - 1xN array of ending joint angles
%       num_steps - Number of steps in the trajectory
%
%   Outputs:
%       q_traj    - (num_steps)xN array of joint angles for the path

    dof = length(q_start);
    q_traj = zeros(num_steps, dof);
    
    % Create a normalized time vector from 0 to 1
    t = linspace(0, 1, num_steps)';
    
    % Calculate polynomial coefficients (for zero vel/accel)
    % s(t) = 10*t^3 - 15*t^4 + 6*t^5
    s_t = 10*(t.^3) - 15*(t.^4) + 6*(t.^5);
    
    % Apply this scaling to each joint
    for i = 1:dof
        delta_q = q_end(i) - q_start(i);
        q_traj(:, i) = q_start(i) + s_t * delta_q;
    end

end

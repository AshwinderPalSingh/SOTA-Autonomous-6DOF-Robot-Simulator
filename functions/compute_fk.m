function [T_final, positions] = compute_fk(robot, q_deg)
% COMPUTE_FK - Computes the forward kinematics for an N-DOF robot.
%
%   Inputs:
%       robot   - The robot parameter struct (from a define_... function)
%       q_deg   - 1xN array of joint angles (in degrees)
%
%   Outputs:
%       T_final   - 4x4 homogeneous transformation from base to end-effector
%       positions - 3x(N+1) array of (x,y,z) coordinates for each joint

    % Validate input
    if length(q_deg) ~= robot.dof
        error('Input angle vector length (%d) does not match robot DOF (%d).', length(q_deg), robot.dof);
    end

    % Convert angles to radians
    q_rad = deg2rad(q_deg);

    % Initialize
    T_total = eye(4); % Cumulative transformation matrix
    
    % positions array stores the (x,y,z) of each joint frame,
    % including the base (frame 0)
    positions = zeros(3, robot.dof + 1);
    positions(:, 1) = [0; 0; 0]; % Base position
    
    % Loop through each joint to compute transformations
    for i = 1:robot.dof
        % Get parameters for this joint
        % Note: D-H theta is the *variable*, so we use q_rad(i)
        % and add the D-H offset (which is 0 for UR5)
        theta_i = q_rad(i); 
        d_i     = robot.d(i);
        a_i     = robot.a(i);
        alpha_i = robot.alpha(i);
        
        % Calculate the transformation for this joint
        T_joint = dh_transform(theta_i, d_i, a_i, alpha_i);
        
        % Update the cumulative transformation from base to current joint (i)
        T_total = T_total * T_joint;
        
        % Store the (x, y, z) position of this joint
        % This is the translation part (last column) of the T matrix
        positions(:, i+1) = T_total(1:3, 4);
    end
    
    % Set the final transformation
    T_final = T_total;

end

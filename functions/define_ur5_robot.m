function robot = define_ur5_robot()
% DEFINE_UR5_ROBOT - Creates a struct with the UR5's D-H parameters.
%
%   All parameters are in (mm) and (radians).
%   Source: Universal Robots UR5 D-H parameters
%
%   Outputs:
%       robot - A struct containing all robot parameters.

    % Degrees of Freedom
    robot.dof = 6;

    % Denavit-Hartenberg (D-H) Parameters (in mm)
    
    % d (link offset) - The distance along the previous z-axis
    robot.d = [89.159, 0, 0, 109.15, 94.65, 82.3];
    
    % a (link length) - The distance along the new x-axis
    % Note: a2 and a3 are negative, which is correct for UR5
    robot.a = [0, -425, -392.2, 0, 0, 0];

    % alpha (twist) - The angle about the new x-axis
    robot.alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
    
end

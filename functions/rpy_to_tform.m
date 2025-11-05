function T = rpy_to_tform(pos, rpy_rad)
% RPY_TO_TFORM - Converts a position vector and RPY angles to a 4x4 T-Matrix.
%
%   Inputs:
%       pos     - 1x3 vector [x, y, z]
%       rpy_rad - 1x3 vector [roll, pitch, yaw] in RADIANS
%
%   Outputs:
%       T       - 4x4 homogeneous transformation matrix

    r = rpy_rad(1); % Roll (about X)
    p = rpy_rad(2); % Pitch (about Y)
    y = rpy_rad(3); % Yaw (about Z)
    
    % Calculate rotation matrices
    Rx = [1, 0, 0; 
          0, cos(r), -sin(r); 
          0, sin(r), cos(r)];
          
    Ry = [cos(p), 0, sin(p); 
          0, 1, 0; 
          -sin(p), 0, cos(p)];
          
    Rz = [cos(y), -sin(y), 0; 
          sin(y), cos(y), 0; 
          0, 0, 1];
          
    % Combined Rotation Matrix (Z-Y-X convention)
    R = Rz * Ry * Rx;
    
    % Create 4x4 Transformation Matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = pos(:); % Ensure it's a column vector

end

function T = dh_transform(theta, d, a, alpha)
% DH_TRANSFORM - Calculates a single Denavit-Hartenberg transformation matrix.
%
%   Inputs:
%       theta - Joint angle (radians)
%       d     - Link offset (D-H parameter)
%       a     - Link length (D-H parameter)
%       alpha - Link twist (D-H parameter)
%
%   Outputs:
%       T     - 4x4 homogeneous transformation matrix

    % Standard D-H Transformation Matrix
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),           cos(alpha),           d;
         0,           0,                    0,                    1];
end

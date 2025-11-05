function [roll_deg, pitch_deg, yaw_deg] = tform_to_rpy(R)
% TFORM_TO_RPY - Converts a 3x3 rotation matrix to Roll, Pitch, Yaw.
%   Uses the Z-Y-X Euler angle convention (Yaw, Pitch, Roll)
%
%   Inputs:
%       R - 3x3 Rotation Matrix
%
%   Outputs:
%       roll_deg  - Roll (rotation about X) in degrees
%       pitch_deg - Pitch (rotation about Y) in degrees
%       yaw_deg   - Yaw (rotation about Z) in degrees

    % Check for gimbal (singularity)
    sy = sqrt(R(1,1)^2 + R(2,1)^2);
    singular = (sy < 1e-6);

    if ~singular
        % Standard case
        x = atan2(R(3,2), R(3,3)); % Roll
        y = atan2(-R(3,1), sy);   % Pitch
        z = atan2(R(2,1), R(1,1)); % Yaw
    else
        % Gimbal lock case
        x = atan2(-R(2,3), R(2,2)); % Roll
        y = atan2(-R(3,1), sy);    % Pitch
        z = 0;                       % Yaw
    end
    
    % Convert to degrees
    roll_deg = rad2deg(x);
    pitch_deg = rad2deg(y);
    yaw_deg = rad2deg(z);
end

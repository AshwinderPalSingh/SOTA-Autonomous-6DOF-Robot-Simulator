function q_sols = solve_ur5_ik(T, robot)
% SOLVE_UR5_IK - Analytical Inverse Kinematics Solver for UR5
%
%   Inputs:
%       T     - 4x4 homogeneous transformation matrix (target pose)
%       robot - The robot parameter struct (must have .d, .a)
%
%   Outputs:
%       q_sols - Nx6 matrix of N joint angle solutions (in radians)
%                Returns empty [] if no solution exists.

    % Get D-H parameters from robot struct
    d = robot.d;
    a = robot.a;
    alpha = robot.alpha; % We need alpha
    
    q_sols = zeros(8, 6); % Pre-allocate for up to 8 solutions
    sol_count = 1;
    
    % --- Standard, robust UR5 IK solver logic ---
    
    T06 = T;
    P06 = T06(1:3, 4);
    R06 = T06(1:3, 1:3);
    
    % --- 1. Wrist Center Position (P05) ---
    P05 = P06 - d(6) * R06(:, 3);
    
    % --- 2. Solve for Theta 1 ---
    phi = atan2(P05(2), P05(1));
    
    % Check for reachability
    arg_acos_1 = d(4) / sqrt(P05(1)^2 + P05(2)^2);
    if abs(arg_acos_1) > 1.001 % Use a small tolerance
        warning('IK Error: Target out of reach (q1).');
        q_sols = []; 
        return;
    end
    arg_acos_1 = max(-1, min(1, arg_acos_1)); % Clamp value
    
    q1_sol_1 = phi + acos(arg_acos_1) + pi/2;
    q1_sol_2 = phi - acos(arg_acos_1) + pi/2;
    
    % --- Iterate through both q1 solutions ---
    for q1 = [q1_sol_1, q1_sol_2]
        
        % --- 3. Solve for Theta 5 ---
        P15_z = P05(1)*sin(q1) - P05(2)*cos(q1);
        
        arg_acos_5 = (P15_z - d(4)) / d(6);
        if abs(arg_acos_5) > 1.001
            continue; % This q1 branch is invalid
        end
        arg_acos_5 = max(-1, min(1, arg_acos_5)); % Clamp
                
        q5_sol_1 = acos(arg_acos_5);
        q5_sol_2 = -acos(arg_acos_5);
        
        % --- Iterate through both q5 solutions ---
        for q5 = [q5_sol_1, q5_sol_2]
            if abs(sin(q5)) < 1e-6 % Check for singularity
                continue; % Skip this solution
            end

            % --- 4. Solve for Theta 6 ---
            T60 = inv(T06);
            q6 = atan2( (-T60(2,1)*sin(q1) + T60(2,2)*cos(q1))/sin(q5), ...
                        ( T60(1,1)*sin(q1) - T60(1,2)*cos(q1))/sin(q5) );
            
            % --- 5. Solve for Theta 3 ---
            T01 = dh_transform(q1, d(1), a(1), alpha(1));
            T45 = dh_transform(q5, d(5), a(5), alpha(5));
            T56 = dh_transform(q6, d(6), a(6), alpha(6));
            
            T14 = inv(T01) * T06 * inv(T45 * T56);
            P14 = T14(1:3, 4); % Position of frame 4 relative to frame 1
            
            arg_acos_3 = (P14(1)^2 + P14(2)^2 - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
            if abs(arg_acos_3) > 1.001
                continue; % This branch is invalid
            end
            arg_acos_3 = max(-1, min(1, arg_acos_3)); % Clamp
            
            q3_sol_1 = acos(arg_acos_3);
            q3_sol_2 = -acos(arg_acos_3);
            
            % --- Iterate through both q3 solutions ---
            for q3 = [q3_sol_1, q3_sol_2]
                
                % --- 6. Solve for Theta 2 ---
                s3 = sin(q3); c3 = cos(q3);
                k1 = a(2) + a(3)*c3;
                k2 = a(3)*s3;
                q2 = atan2(P14(2), P14(1)) - atan2(k2, k1);
                
                % --- 7. Solve for Theta 4 ---
                T13 = dh_transform(q2, d(2), a(2), alpha(2)) * ...
                      dh_transform(q3, d(3), a(3), alpha(3));
                T34 = inv(T13) * T14;
                q4 = atan2(T34(2,1), T34(1,1));
                
                % --- 8. Store the complete solution ---
                q_sols(sol_count, :) = [q1, q2, q3, q4, q5, q6];
                sol_count = sol_count + 1;
            end
        end
    end
    
    q_sols = q_sols(1:sol_count-1, :); % Trim unused rows

end

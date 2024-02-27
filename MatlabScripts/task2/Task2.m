clc;
clear;

run("config.m");

T_3d = [100, 100, 100];
T_angle_initial_guess = -pi;
L12 = 100;
L23 = 100;
L34 = 100;
L45 = 100;
joint1_limit = [-pi, pi];
joint2_limit = [-pi, pi];
joint3_limit = [-pi, pi];
joint4_limit = [-pi, pi];

% given desired position and calculate the joint angles
[joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ...
    IK(T_3d, T_angle_initial_guess, L12, L23, L34, L45, joint1_limit, joint2_limit, joint3_limit, joint4_limit);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,joint1_angle/0.088);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,joint2_angle/0.088);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,joint3_angle/0.088);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,joint4_angle/0.088);

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ...
    IK(T_3d, T_angle_initial_guess, L12, L23, L34, L45, joint1_limit, joint2_limit, joint3_limit, joint4_limit)

    T_angle_guess_step = 0.1;
    old_joint3_angle = 0;
    iteration = 1;
    avoid_sigularity_mode = 0;

    joint1_angle = atan2(T_3d(2), T_3d(1));
    T_2d = [sqrt(T_3d(1)^2 + T_3d(2)^2), T_3d(3)-L12];

    while iteration <= 4000
        %get the position of C
        pos_C = [T_2d(1) - L45 * cos(T_angle_initial_guess), T_2d(2) - L45 * sin(T_angle_initial_guess)];
        L24 = sqrt((pos_C(1))^2 + (pos_C(2))^2);

        % Check AC First    
        if L24 > L12 + L23
            T_angle_initial_guess = T_angle_initial_guess + deg2rad(T_angle_guess_step);
            iteration = iteration + 1;
            % fprintf('AC is greater than AB + BC in iteration: %d\n', iteration);
            continue;
        else
            % check no joint angle is out of limit
            L24_angle = atan2(pos_C(2), pos_C(1));
            joint3_angle = acos((L24^2 - L23^2 - L34^2)/(2*L23*L34));
            %where sigularity happens 
            % avoid the singularity
            if(joint3_angle>0)
                joint2_angle = L24_angle - asin(L34*sin(joint3_angle)/L24);
            else
                joint2_angle = L24_angle + asin(L34*sin(joint3_angle)/L24);
            end
            joint4_angle = T_angle_initial_guess-joint2_angle-joint3_angle;

            if joint2_angle < joint2_limit(1) || joint2_angle > joint2_limit(2) || ...
                joint3_angle < joint3_limit(1) || joint3_angle > joint3_limit(2) || ...
                joint4_angle < joint4_limit(1) || joint4_angle > joint4_limit(2)

                fprintf('Joint angle out of limit in iteration: %d\n', iteration);
                T_angle_initial_guess = T_angle_initial_guess + deg2rad(T_angle_guess_step);
                iteration = iteration + 1;
                continue;
            else
                fprintf('T_angle_initial_guess: %f\n', T_angle_initial_guess);
                fprintf('joint2_angle: %f, joint3_angle: %f, joint4_angle: %f\n', ...
                    (joint2_angle), (joint3_angle), (joint4_angle));
                break;
%                 if avoid_sigularity_mode == 1
%                     if sign(joint3_angle) == sign(old_joint3_angle)
%                         break;
%                     else
%                         old_joint3_angle = joint3_angle;
%                         T_angle_initial_guess = T_angle_initial_guess + deg2rad(T_angle_guess_step);
%                         iteration = iteration + 1;
%                         continue;
%                     end
%                 else
%                     break;
%                 end
            end
        end
    end

    if iteration > 4000
        error('The iteration is greater than 4000');
    end
end
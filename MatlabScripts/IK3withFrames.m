clear

joint1_limit = [deg2rad(0), deg2rad(360)];
joint2_limit = [deg2rad(100), deg2rad(330)];
joint3_limit = [deg2rad(30), deg2rad(330)];
joint4_limit = [deg2rad(30), deg2rad(330)];

L1 = 77;
L2 = 130;
L3 = 124;
L4 = 126;

thetaOffset = asin(24/130);
% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [L1, 0, 0, 0];  % Joint offsets

link_colors = {'m-','k-', 'b-', 'g-'};

% circle radius
radius = 150;  %radius
center = [0, 0];  % center
num_points = 80;
theta = linspace(0, 2*pi, num_points);
x1 = center(1) + radius * cos(theta);
y1 = center(2) + radius * sin(theta);
z1 = ones(num_points)*77;

x2 = linspace(-200, 200, num_points);
y2 = linspace(200, 200, num_points);
z2 = ones(num_points)*77;

% 定义正方形的边长和点的个数
side_length = 100;
num_points_per_side = num_points/4;

%left bottom corner
C_1 = [50, -100];
C_2 = [-120, -120];
C_3 = [-100, -100];


[x3_1, y3_1] = generateSquarePoints(C_1, side_length, num_points_per_side);
[x3_2, y3_2] = generateSquarePoints(C_2, side_length, num_points_per_side);
[x3_3, y3_3] = generateSquarePoints(C_1, side_length, num_points_per_side);

z3_1 = -ones(1, num_points)*200;
z3_2 = ones(1, num_points)*200;
z3_3 = -ones(1, num_points)*200;
% T_3d = [200, 50, 77];

x4 = linspace(250, 250, num_points);
y4 = linspace(-100, 100, num_points);
z4 = linspace(77, 77, num_points);

% fprintf('theta1: %f, theta2: %f, theta3: %f, theta4: %f\n', theta1, theta2, theta3, theta4);



% Initialize figure
frameLength = 30;
default_fig_position = [500, 300, 1200, 1000]; % [left, bottom, width, height]
figure('Position', default_fig_position);
hold on;
axis equal;
axis([-300, 300, -300, 300, -300, 300]);
% Set the view perspective for 3D
view(3);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('4-DOF Robot Arm Movement');


% Loop through each point and calculate end-effector position
num_points_total = num_points * 3;

x3 = [z3_1 x3_2 y3_3];
y3 = [y3_1 y3_2 z3_3];
z3 = [x3_1 z3_2 x3_3];

% num_points_total = 1;
% for i = 1:num_points
for i = 1:num_points_total
    
    % plot3(x4, y4, z4, 'r-', 'LineWidth', 3);

    %circle
    % T_3d = [x1(i), y1(i), z1(i)];

    %line
    % T_3d = [x4(i), y4(i), z4(i)];

    %square
    T_3d = [x3(i), y3(i), z3(i)];
    % T_3d = [-200,0,50];
    % T_3d = [300,0,100];
    % T_3d = [-200,0,100];
    % T_3d = [0,0,100];
 
    [theta1, theta2, theta3, theta4] = IK(T_3d, 0, L1, L2, L3, L4, ...
        joint1_limit, joint2_limit, joint3_limit, joint4_limit);
    [theta1, theta2, theta3, theta4] = ServoAnglesToIkAngles(theta1, theta2, theta3, theta4);
    
    % fprintf('Joint1 ik angle: %f\n', rad2deg(theta1));
    % fprintf('Joint2 ik angle: %f\n', rad2deg(theta2));
    % fprintf('Joint3 ik angle: %f\n', rad2deg(theta3));
    % fprintf('Joint4 ik angle: %f\n', rad2deg(theta4));


    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup', '-or', 'Type', 'quiver');

    % Exclude the end effector from the list of objects to delete
    end_effector_object = findobj(all_objects, 'Tag', 'EndEffector');
    objects_to_delete = setdiff(all_objects, end_effector_object);

    % Delete objects other than the end effector
    delete(objects_to_delete);

    % Transformation matrices
    T1 = dh_matrix_3d(a(1), alpha(1), d(1), theta1);
    T2 = dh_matrix_3d(a(2), alpha(2), d(2), theta2);
    T3 = dh_matrix_3d(a(3), alpha(3), d(3), theta3);
    T4 = dh_matrix_3d(a(4), alpha(4), d(4), theta4);

    % Calculate link end points
    link1_end = T1 * [0; 0; 0; 1];
    link2_end = T1 * T2 * [0; 0; 0; 1];
    link3_end = T1 * T2 * T3 * [0; 0; 0; 1];
    end_effector_position = T1 * T2 * T3 * T4 * [0; 0; 0; 1];

    % Plot robot arm links with different colors
    plot3([0, link1_end(1), link2_end(1), link3_end(1), end_effector_position(1)], ...
          [0, link1_end(2), link2_end(2), link3_end(2), end_effector_position(2)], ...
          [0, link1_end(3), link2_end(3), link3_end(3), end_effector_position(3)], link_colors{1}, 'LineWidth', 2);
      
    % Plot each link with a different color
    plot3([0, link1_end(1)], [0, link1_end(2)], [0, link1_end(3)], link_colors{1}, 'LineWidth', 2);
    plot3([link1_end(1), link2_end(1)], [link1_end(2), link2_end(2)], [link1_end(3), link2_end(3)], link_colors{2}, 'LineWidth', 2);
    plot3([link2_end(1), link3_end(1)], [link2_end(2), link3_end(2)], [link2_end(3), link3_end(3)], link_colors{3}, 'LineWidth', 2);
    plot3([link3_end(1), end_effector_position(1)], [link3_end(2), end_effector_position(2)], [link3_end(3), end_effector_position(3)], link_colors{4}, 'LineWidth', 2);
      
    % Plot end of each link as a blue solid circle
    plot3([link1_end(1), link2_end(1), link3_end(1)], ...
          [link1_end(2), link2_end(2), link3_end(2)], ...
          [link1_end(3), link2_end(3), link3_end(3)], 'bo-', 'MarkerSize', 8);
    
    if (i<=num_points_total/3)
        color = 'ro-';
    elseif (i<=num_points_total*2/3)
        color = 'ko-';
    else
        color = 'bo-';
    end
    % Plot end effector as a red hollow circle
    plot3(end_effector_position(1), end_effector_position(2), end_effector_position(3), color, 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'Tag', 'EndEffector');
    
    % Plot XYZ frames at each joint
    % Joint 1
    quiver3(0, 0, 0, frameLength, 0, 0, 'g', 'LineWidth', 2); % X-axis
    quiver3(0, 0, 0, 0, frameLength, 0, 'b', 'LineWidth', 2); % Y-axis
    quiver3(0, 0, 0, 0, 0, frameLength, 'r', 'LineWidth', 2); % Z-axis

    % Joint 2
    joint2_origin = T1 * [0; 0; 0; 1];
    joint2_x_axis = T1 * [frameLength; 0; 0; 1];
    joint2_y_axis = T1 * [0; frameLength; 0; 1];
    joint2_z_axis = T1 * [0; 0; frameLength; 1];
    quiver3(joint2_origin(1), joint2_origin(2), joint2_origin(3), joint2_x_axis(1) - joint2_origin(1), joint2_x_axis(2) - joint2_origin(2), joint2_x_axis(3) - joint2_origin(3), 'g', 'LineWidth', 2); % X-axis
    quiver3(joint2_origin(1), joint2_origin(2), joint2_origin(3), joint2_y_axis(1) - joint2_origin(1), joint2_y_axis(2) - joint2_origin(2), joint2_y_axis(3) - joint2_origin(3), 'b', 'LineWidth', 2); % Y-axis
    quiver3(joint2_origin(1), joint2_origin(2), joint2_origin(3), joint2_z_axis(1) - joint2_origin(1), joint2_z_axis(2) - joint2_origin(2), joint2_z_axis(3) - joint2_origin(3), 'r', 'LineWidth', 2); % Z-axis

    % Joint 3
    joint3_origin = T1 * T2 * [0; 0; 0; 1];
    joint3_x_axis = T1 * T2 * [frameLength; 0; 0; 1];
    joint3_y_axis = T1 * T2 * [0; frameLength; 0; 1];
    joint3_z_axis = T1 * T2 * [0; 0; frameLength; 1];
    quiver3(joint3_origin(1), joint3_origin(2), joint3_origin(3), joint3_x_axis(1) - joint3_origin(1), joint3_x_axis(2) - joint3_origin(2), joint3_x_axis(3) - joint3_origin(3), 'g', 'LineWidth', 2); % X-axis
    quiver3(joint3_origin(1), joint3_origin(2), joint3_origin(3), joint3_y_axis(1) - joint3_origin(1), joint3_y_axis(2) - joint3_origin(2), joint3_y_axis(3) - joint3_origin(3), 'b', 'LineWidth', 2); % Y-axis
    quiver3(joint3_origin(1), joint3_origin(2), joint3_origin(3), joint3_z_axis(1) - joint3_origin(1), joint3_z_axis(2) - joint3_origin(2), joint3_z_axis(3) - joint3_origin(3), 'r', 'LineWidth', 2); % Z-axis

    % Joint 4
    joint4_origin = T1 * T2 * T3 * [0; 0; 0; 1];
    joint4_x_axis = T1 * T2 * T3 * [frameLength; 0; 0; 1];
    joint4_y_axis = T1 * T2 * T3 * [0; frameLength; 0; 1];
    joint4_z_axis = T1 * T2 * T3 * [0; 0; frameLength; 1];
    quiver3(joint4_origin(1), joint4_origin(2), joint4_origin(3), joint4_x_axis(1) - joint4_origin(1), joint4_x_axis(2) - joint4_origin(2), joint4_x_axis(3) - joint4_origin(3), 'g', 'LineWidth', 2); % X-axis
    quiver3(joint4_origin(1), joint4_origin(2), joint4_origin(3), joint4_y_axis(1) - joint4_origin(1), joint4_y_axis(2) - joint4_origin(2), joint4_y_axis(3) - joint4_origin(3), 'b', 'LineWidth', 2); % Y-axis
    quiver3(joint4_origin(1), joint4_origin(2), joint4_origin(3), joint4_z_axis(1) - joint4_origin(1), joint4_z_axis(2) - joint4_origin(2), joint4_z_axis(3) - joint4_origin(3), 'r', 'LineWidth', 2); % Z-axis
    
   % Calculate end effector transformation matrix
    T_end_effector = T1 * T2 * T3 * T4;

    % Extract end effector position and orientation from transformation matrix
    end_effector_position = T_end_effector(1:3, 4);
    end_effector_x_axis = T_end_effector * [frameLength; 0; 0; 1];
    end_effector_y_axis = T_end_effector * [0; frameLength; 0; 1];
    end_effector_z_axis = T_end_effector * [0; 0; frameLength; 1];

    % Plot XYZ frame for end effector
    quiver3(end_effector_position(1), end_effector_position(2), end_effector_position(3), ...
        end_effector_x_axis(1) - end_effector_position(1), ...
        end_effector_x_axis(2) - end_effector_position(2), ...
        end_effector_x_axis(3) - end_effector_position(3), 'r', 'LineWidth', 2); % X-axis
    quiver3(end_effector_position(1), end_effector_position(2), end_effector_position(3), ...
        end_effector_y_axis(1) - end_effector_position(1), ...
        end_effector_y_axis(2) - end_effector_position(2), ...
        end_effector_y_axis(3) - end_effector_position(3), 'g', 'LineWidth', 2); % Y-axis
    quiver3(end_effector_position(1), end_effector_position(2), end_effector_position(3), ...
        end_effector_z_axis(1) - end_effector_position(1), ...
        end_effector_z_axis(2) - end_effector_position(2), ...
        end_effector_z_axis(3) - end_effector_position(3), 'b', 'LineWidth', 2); % Z-axis
   
    % Update the figure window
    drawnow;

    % Pause to create animation effect
    pause(0.01);
end

hold off;


function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ...
    IK(T_3d, end_ang, L12, L23, L34, L45, joint1_limit, joint2_limit, joint3_limit, joint4_limit)
    thetaOffset = asin(24/130);
    iteration = 0;
    solutions_array = [];
    stepSize = deg2rad(0.1);
    % [joint1_limit, joint2_limit, joint3_limit, joint4_limit] = servoLimitesToIkLimits(joint1_limit, joint2_limit, joint3_limit, joint4_limit);
    fprintf('joint1_limit: %f, %f\n', rad2deg(joint1_limit(1)), rad2deg(joint1_limit(2)));
    fprintf('joint2_limit: %f, %f\n', rad2deg(joint2_limit(1)), rad2deg(joint2_limit(2)));
    fprintf('joint3_limit: %f, %f\n', rad2deg(joint3_limit(1)), rad2deg(joint3_limit(2)));
    fprintf('joint4_limit: %f, %f\n', rad2deg(joint4_limit(1)), rad2deg(joint4_limit(2)));
    while iteration < 4000 && size(solutions_array, 1) < 3
        iteration = iteration + 1;
        fprintf('Iteration: %d\n', iteration);
        fprintf('end_ang: %f\n', rad2deg(end_ang));
        joint1_angle = atan2(T_3d(2), T_3d(1));
        T_2d = [sqrt(T_3d(1)^2 + T_3d(2)^2), T_3d(3)-L12]; 
        pos_4 = [T_2d(1) - cos(end_ang)*L45, T_2d(2) - sin(end_ang)*L45];
        L24_angle = atan2(pos_4(2), pos_4(1));
        L24 = sqrt(pos_4(1)^2 + pos_4(2)^2);
        fprintf('pos_4: %f, %f\n', pos_4(1), pos_4(2));
        fprintf('L24_angle: %f\n', rad2deg(L24_angle));
        fprintf('L24: %f\n', L24); 
        if L24 > L23 + L34
            fprintf('L24 is too long\n');
            end_ang = end_ang + stepSize;
            continue;
        end
        joint3_angle = -acos((L24^2 - L23^2 - L34^2)/(2*L23*L34));
        joint2_angle = L24_angle + acos((L23^2 + L24^2 - L34^2)/(2*L23*L24));
        joint4_angle = end_ang - joint2_angle - joint3_angle;

        [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ikAnglesToServoAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle);
        % if joint1_angle < joint1_limit(1) || joint1_angle > joint1_limit(2) || ...
        %     joint2_angle < joint2_limit(1) || joint2_angle > joint2_limit(2) || ...
        %     joint3_angle < joint3_limit(1) || joint3_angle > joint3_limit(2) || ...
        %     joint4_angle < joint4_limit(1) || joint4_angle > joint4_limit(2)
        %     fprintf('Joint angle out of limit\n');
        %     fprintf('joint1_angle: %f\n', rad2deg(joint1_angle));
        %     fprintf('joint2_angle: %f\n', rad2deg(joint2_angle));
        %     fprintf('joint3_angle: %f\n', rad2deg(joint3_angle));
        %     fprintf('joint4_angle: %f\n', rad2deg(joint4_angle));
        %     end_ang = end_ang + stepSize;
        %     continue;
        % end
        
        solutions_array = [solutions_array; [joint1_angle, joint2_angle, joint3_angle, joint4_angle]];
    end
    if size(solutions_array) == 0
        error('No solution found');
    end
    fprintf('Number of solutions: %d\n', size(solutions_array, 1));
    fprintf('First solution:\ntheta1:%f\ntheta2:%f\ntheta3:%f\ntheta4:%f\n', ...
        rad2deg(solutions_array(1,1)), rad2deg(solutions_array(1,2) - thetaOffset), rad2deg(solutions_array(1,3) + thetaOffset), rad2deg(solutions_array(1,4)));
end

function dh_matrix_2d = dh_matrix_2d(a, alpha, d, theta)
    % 计算DH参数对应的转换矩阵（2D）
    dh_matrix_2d = [cos(theta), -sin(theta), a*cos(theta);
                            sin(theta), cos(theta), a*sin(theta);
                            0, 0, 1];
end

function dh_matrix_3d = dh_matrix_3d(a, alpha, d, theta)
    dh_matrix_3d = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

function [joint1_limit_new, joint2_limit_new, joint3_limit_new, joint4_limit_new] = servoLimitesToIkLimits(joint1_limit, joint2_limit, joint3_limit, joint4_limit)
    joint1_limit_new = [joint1_limit(1), joint1_limit(2)];
    joint2_limit_new = [joint2_limit(1) - pi/2, joint2_limit(2) - pi/2];
    joint3_limit_new = [joint3_limit(1) - pi, joint3_limit(2) - pi];
    joint4_limit_new = [joint4_limit(1) - pi, joint4_limit(2) - pi];
end

function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ikAnglesToServoAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
    fprintf('ik_joint1_angle: %f\n', rad2deg(joint1_angle));
    fprintf('ik_joint2_angle: %f\n', rad2deg(joint2_angle));
    fprintf('ik_joint3_angle: %f\n', rad2deg(joint3_angle));
    fprintf('ik_joint4_angle: %f\n', rad2deg(joint4_angle));
    joint1_angle = joint1_angle + 2*pi;
    joint2_angle = 3*pi/2 - joint2_angle;
    joint3_angle = -joint3_angle + pi/2 ;
    joint4_angle = pi - joint4_angle;
    fprintf('servo_joint1_angle: %f\n', rad2deg(joint1_angle));
    fprintf('servo_joint2_angle: %f\n', rad2deg(joint2_angle));
    fprintf('servo_joint3_angle: %f\n', rad2deg(joint3_angle));
    fprintf('servo_joint4_angle: %f\n', rad2deg(joint4_angle));
end

function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ServoAnglesToIkAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
    joint1_angle = joint1_angle - 2*pi;
    joint2_angle = -joint2_angle + 3*pi/2;
    joint3_angle = -joint3_angle + pi/2;
    joint4_angle = -joint4_angle + pi;
end

function [x, y] = generateSquarePoints(C, side_length, num_points_per_side)
    x = [];
    y = [];

    % bottom
    x = [x, linspace(C(1), C(1)+side_length, num_points_per_side)];
    y = [y, linspace(C(2), C(2), num_points_per_side)];

    % right
    x = [x, C(1)+side_length * ones(1, num_points_per_side)];
    y = [y, linspace(C(2),C(2) + side_length, num_points_per_side)];

    % top
    x = [x, linspace(C(1) + side_length, C(1), num_points_per_side)];
    y = [y, C(2) + side_length * ones(1, num_points_per_side)];

    % left
    x = [x, linspace(C(1), C(1), num_points_per_side)];
    y = [y, linspace(C(2) + side_length, C(2), num_points_per_side)];
end
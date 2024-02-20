clear

AB = 100;
BC = 100;
CT = 100;

jointA_limit = [-pi, pi];
jointB_limit = [-pi, pi];
jointC_limit = [-pi, pi];



L1 = 43;
L2 = 100;
L3 = 100;
L4 = 100;

thetaOffset = asin(24/130);
% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [77, 0, 0, 0];  % Joint offsets

link_colors = {'m-','k-', 'b-', 'g-'};

% circle radius
radius = 100;  %radius
center = [100, 100];  % center
num_points = 400;
theta = linspace(0, 2*pi, num_points);
x1 = center(1) + radius * cos(theta);
y1 = center(2) + radius * sin(theta);

x2 = linspace(-200, 200, num_points);
y2 = linspace(200, 200, num_points);
z2 = ones(num_points)*77;

% 定义正方形的边长和点的个数
side_length = 400;
num_points_per_side = num_points/4;

%left bottom corner
C = [-200, -200];

% 创建正方形边上的点集
x3 = [];
y3 = [];

% 添加下边
x3 = [x3, linspace(C(1), C(1)+side_length, num_points_per_side)];
y3 = [y3, linspace(C(2), C(2), num_points_per_side)];

% 添加右边
x3 = [x3, C(1)+side_length * ones(1, num_points_per_side)];
y3 = [y3, linspace(C(2),C(2) + side_length, num_points_per_side)];

% 添加上边
x3 = [x3, linspace(C(1) + side_length, C(1), num_points_per_side)];
y3 = [y3, C(2) + side_length * ones(1, num_points_per_side)];

% 添加左边
x3 = [x3, linspace(C(1), C(1), num_points_per_side)];
y3 = [y3, linspace(C(2) + side_length, C(2), num_points_per_side)];

% T_3d = [200, 50, 77];

x4 = linspace(150, 150, num_points);
y4 = linspace(-50, 50, num_points);
z4 = linspace(77, 77, num_points);

% fprintf('theta1: %f, theta2: %f, theta3: %f, theta4: %f\n', theta1, theta2, theta3, theta4);


default_fig_position = [500, 300, 1200, 1000]; % [left, bottom, width, height]

% Initialize figure
figure('Position', default_fig_position);
hold on;
axis equal;
axis([-500, 500, -500, 500, -500, 500]);
% Set the view perspective for 3D
view(3);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('4-DOF Robot Arm Movement');

% Loop through each point and calculate end-effector position
for i = 1:num_points
    
    plot3(x4, y4, z4, 'r-', 'LineWidth', 3);

    T_3d = [x4(i), y4(i), z4(i)];
    theta1 = atan2(T_3d(2), T_3d(1));
    T_2d = [sqrt(T_3d(1)^2 + T_3d(2)), T_3d(3)-d(1)];
    T_angle_initial_guess = -pi;
    [theta2, theta3, theta4] = IK(T_2d, T_angle_initial_guess, AB, BC, CT, jointA_limit, jointB_limit, jointC_limit);


    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup');

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
    
    % Plot end effector as a red hollow circle
    plot3(end_effector_position(1), end_effector_position(2), end_effector_position(3), 'ro-', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'Tag', 'EndEffector');
    
    % Update the figure window
    drawnow;

    % Pause to create animation effect
    pause(0.01);
end

hold off;






function [jointA_angle, jointB_angle, jointC_angle] = IK(T_2d, T_angle_initial_guess, AB, BC, CT, jointA_limit, jointB_limit, jointC_limit)
    iteration = 1;
    while iteration <= 4000
        
        %get the position of C
        pos_C = [T_2d(1) - CT * cos(T_angle_initial_guess), T_2d(2) - CT * sin(T_angle_initial_guess)];
        AC = sqrt((pos_C(1))^2 + (pos_C(2))^2);
        % fprintf('pos_C: %d, %d\n', pos_C(1), pos_C(2));
        % fprintf('The AC: %d, AB + BC: %d\n', AC, AB + BC);

        % Check AC First    
        if AC > AB + BC
            T_angle_initial_guess = T_angle_initial_guess + deg2rad(0.1);
            iteration = iteration + 1;
            % fprintf('AC is greater than AB + BC in iteration: %d\n', iteration);
            continue;
        else
            % check no joint angle is out of limit
            AC_angle = atan2(pos_C(2), pos_C(1));
            jointB_angle = acos((AC^2 - AB^2 - BC^2)/(2*AB*BC));
            if(jointB_angle>0)
                jointA_angle = AC_angle - asin(BC*sin(jointB_angle)/AC);
            else
                jointA_angle = AC_angle + asin(BC*sin(jointB_angle)/AC);
            end
            jointC_angle = T_angle_initial_guess-jointA_angle-jointB_angle;
            % jointA_angle = asin(S/(AB*AC)) + AC_angle;
            % jointB_angle = asin(S/(AB*BC));
            % jointC_angle = T_angle_initial_guess - jointA_angle - jointB_angle;
            
            if jointA_angle < jointA_limit(1) || jointA_angle > jointA_limit(2) || ...
                jointB_angle < jointB_limit(1) || jointB_angle > jointB_limit(2) || ...
                jointC_angle < jointC_limit(1) || jointC_angle > jointC_limit(2)
                fprintf('Joint angle out of limit in iteration: %d\n', iteration);
                T_angle_initial_guess = T_angle_initial_guess + deg2rad(1);
                iteration = iteration + 1;
            else
                fprintf('T_angle_initial_guess: %f\n', T_angle_initial_guess);
                fprintf('jointA_angle: %f, jointB_angle: %f, jointC_angle: %f\n', ...
                    (jointA_angle), (jointB_angle), (jointC_angle));
                break;
            end
        end
    end

    if iteration > 4000
        error('The iteration is greater than 4000');
    end
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

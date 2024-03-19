clear

L1 = 77;
L2 = 130;
L3 = 124;
L4 = 126;

% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [L1, 0, 0, 0];  % Joint offsets

link_colors = {'m-','k-', 'b-', 'g-'};
num_points = 400;

theta1s = linspace(0, pi/2, num_points/4);
theta2s = linspace(0, pi/2, num_points/4);
theta3s = linspace(0, pi/2, num_points/4);
theta4s = linspace(0, pi/2, num_points/4);

% Initialize figure
frameLength = 50;
default_fig_position = [500, 300, 1200, 1000]; % [left, bottom, width, height]
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

for i = 1:num_points
    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup', '-or', 'Type', 'quiver');

    % Exclude the end effector from the list of objects to delete
    end_effector_object = findobj(all_objects, 'Tag', 'EndEffector');
    objects_to_delete = setdiff(all_objects, end_effector_object);

    % Delete objects other than the end effector
    delete(objects_to_delete);

    if i <= num_points/4
        theta1 = theta1s(i);
        theta2 = 0;
        theta3 = 0;
        theta4 = 0;
        color = 'ro-';
    elseif i <= num_points/2
        theta1 = theta1s(end);
        theta2 = theta2s(i - num_points/4);
        theta3 = 0;
        theta4 = 0;
        color = 'go-';
    elseif i <= 3*num_points/4
        theta1 = theta1s(end);
        theta2 = theta2s(end);
        theta3 = theta3s(i - num_points/2);
        theta4 = 0;
        color = 'bo-';
    else
        theta1 = theta1s(end);
        theta2 = theta2s(end);
        theta3 = theta3s(end);
        theta4 = theta4s(i - 3*num_points/4);
        color = 'mo-';
    end

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
        end_effector_x_axis(3) - end_effector_position(3), 'g', 'LineWidth', 2); % X-axis
    quiver3(end_effector_position(1), end_effector_position(2), end_effector_position(3), ...
        end_effector_y_axis(1) - end_effector_position(1), ...
        end_effector_y_axis(2) - end_effector_position(2), ...
        end_effector_y_axis(3) - end_effector_position(3), 'b', 'LineWidth', 2); % Y-axis
    quiver3(end_effector_position(1), end_effector_position(2), end_effector_position(3), ...
        end_effector_z_axis(1) - end_effector_position(1), ...
        end_effector_z_axis(2) - end_effector_position(2), ...
        end_effector_z_axis(3) - end_effector_position(3), 'r', 'LineWidth', 2); % Z-axis
   
    % Update the figure window
    drawnow;

    % Pause to create animation effect
    pause(0.01);
end

hold off;


function dh_matrix_3d = dh_matrix_3d(a, alpha, d, theta)
    dh_matrix_3d = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
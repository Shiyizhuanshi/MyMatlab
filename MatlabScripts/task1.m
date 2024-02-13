clear

L1 = 5;
L2 = 5;
L3 = 5;
L4 = 3;

% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [0, 0, 0, 0];  % Joint offsets

%link_colors = {'b-', 'g-', 'm-', 'c-'};
link_colors = {'m-','k-', 'b-', 'g-'};

% Number of points for visualization
num_points = 100;

theta1_desire_start = 0;
theta2_desire_start = 0;
theta3_desire_start = 0;
theta4_desire_start = 0;

theta1_desire_end = pi;
theta2_desire_end = pi/2;
theta3_desire_end = -pi/2;
theta4_desire_end = -pi/2;



% Initialize joint angles
theta1 = linspace(theta1_desire_start, theta1_desire_end, num_points);
theta2 = linspace(theta2_desire_start, theta2_desire_end, num_points);
theta3 = linspace(theta3_desire_start, theta3_desire_end, num_points);
theta4 = linspace(theta4_desire_start, theta4_desire_end, num_points);


default_fig_position = [500, 300, 1200, 1000]; % [left, bottom, width, height]

% Initialize figure
figure('Position', default_fig_position);
hold on;
axis equal;
axis([-15, 15, -15, 15, -15, 15]);
% Set the view perspective for 3D
view(3);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('4-DOF Robot Arm Movement');

% Loop through each point and calculate end-effector position
for i = 1:num_points

    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup');

    % Exclude the end effector from the list of objects to delete
    end_effector_object = findobj(all_objects, 'Tag', 'EndEffector');
    objects_to_delete = setdiff(all_objects, end_effector_object);

    % Delete objects other than the end effector
    delete(objects_to_delete);

    % Transformation matrices
    T1 = dh_matrix(a(1), alpha(1), d(1), theta1(i));
    T2 = dh_matrix(a(2), alpha(2), d(2), theta2(i));
    T3 = dh_matrix(a(3), alpha(3), d(3), theta3(i));
    T4 = dh_matrix(a(4), alpha(4), d(4), theta4(i));

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
    pause(0.1);
end

hold off;



function A = dh_matrix(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

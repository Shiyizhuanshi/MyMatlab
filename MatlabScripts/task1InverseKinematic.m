clear

L1 = 43;
L2 = 130;
L3 = 124;
L4 = 126;

thetaOffset = asin(24/130);
% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [77, 0, 0, 0];  % Joint offsets

% Specify the parameters of the desired circle in 3D space
radius = 126;  % Adjust the radius as needed
center = [0, 0, 457];  % Adjust the center as needed

% Number of points for visualization
num_points = 100;

% Calculate joint angles using inverse kinematics for the given circle
theta = linspace(0, 2*pi, num_points);
theta1 = zeros(1, num_points);
theta2 = zeros(1, num_points);
theta3 = zeros(1, num_points);
theta4 = zeros(1, num_points);



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

theta_circle = linspace(0, 2*pi, num_points);
x_circle = radius * cos(theta_circle) + center(1);
y_circle = radius * sin(theta_circle) + center(2);
z_circle = ones(1, num_points) * center(3);
plot3( x_circle , y_circle ,z_circle , 'r-', 'LineWidth', 2, 'Tag', 'desiredCircle');

for i = 1:num_points
    % Desired position
    x = center(1) + radius*cos(theta(i));
    y = center(2) + radius*sin(theta(i));
    z = center(3);

    % Inverse kinematics
    theta1(i) = atan2(y, x);

    r = sqrt(x^2 + y^2);
    s = z - d(1);
    D = (r^2 + s^2 - a(2)^2 - a(3)^2) / (2*a(2)*a(3));

    if D > 1 || D < -1
        fprintf('The desired position is not reachable for theta = %f\n', theta(i));
        continue;
    end

    theta3(i) = atan2(sqrt(1-D^2), D);

    k1 = a(2) + a(3)*cos(theta3(i));
    k2 = a(3)*sin(theta3(i));
    theta2(i) = atan2(s, r) - atan2(k2, k1);

    theta4(i) = thetaOffset - theta1(i) - theta2(i) - theta3(i);
end

link_colors = {'m-','k-', 'b-', 'g-'};
% Loop through each point and calculate end-effector position
for i = 1:num_points

    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup');

    % Exclude the end effector from the list of objects to delete
    end_effector_object1 = findobj(all_objects, 'Tag', 'EndEffector');
    end_effector_object2 = findobj(all_objects, 'Tag', 'desiredCircle');
    objects_to_delete = setdiff(all_objects, [end_effector_object1; end_effector_object2]);

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
    pause(0.01);
end

hold off;



function A = dh_matrix(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

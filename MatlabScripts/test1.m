% Define the robot parameters
L1 = 0.3; % length of link 1
L2 = 0.3; % length of link 2

% Define the trajectory
t = (0:0.2:10)';
count = length(t);
center = [0.3, 0.1];
radius = 0.15;
theta = 3*t * (2 * pi / t(end));
points = center + radius * [cos(theta), sin(theta)];

% Pre-allocate joint angles matrix
qs = zeros(count, 2);

% Loop through the trajectory to perform inverse kinematics
for i = 1:count
    x = points(i, 1);
    y = points(i, 2);

    % Inverse Kinematics
    D = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    q2 = atan2(-sqrt(1 - D^2), D);
    q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

    % Store the joint angles
    qs(i, :) = [q1, q2];
end

% Animation
figure
hold on
axis equal
grid on
for i = 1:count
    cla
    % Forward Kinematics
    x1 = L1 * cos(qs(i, 1));
    y1 = L1 * sin(qs(i, 1));

    x2 = x1 + L2 * cos(qs(i, 1) + qs(i, 2));
    y2 = y1 + L2 * sin(qs(i, 1) + qs(i, 2));

    % Plot robot configuration
    plot([0, x1, x2], [0, y1, y2], 'o-', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(x2, y2, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');  

    % Plot trajectory(circle)
    plot(points(:, 1), points(:, 2), 'k');
    
    % Adjust the plot
    axis([-0.1 0.7 -0.3 0.5]);
    drawnow;

    % Pause to control the animation speed
    pause(0.1);
end

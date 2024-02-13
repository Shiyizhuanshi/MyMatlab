% Example end-effector position and orientation
x = 2;
y = 1;
z = 3;
phi = pi/4; % Example orientation in radians

% Call the inverse_kinematics_and_plot function
joint_angles = inverse_kinematics_and_plot(x, y, z, phi);



function joint_angles = inverse_kinematics_and_plot(x, y, z, phi)
    % Link lengths (adjust according to your robot arm)
    L1 = 5;
    L2 = 5;
    L3 = 5;
    L4 = 3;

    % Calculate inverse kinematics
    joint_angles = inverse_kinematics(x, y, z, phi, L1, L2, L3, L4);

    % Plot the robot arm
    plot_robot_arm(joint_angles, L1, L2, L3, L4);
end

function joint_angles = inverse_kinematics(x, y, z, phi, L1, L2, L3, L4)
    % Example equations (replace with actual equations):
    theta1 = atan2(y, x);
    d = sqrt(x^2 + y^2) - L1;
    theta2 = acos((d^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3));
    theta3 = atan2(z, d) - atan2(L3 * sin(theta2), L2 + L3 * cos(theta2));
    theta4 = phi - theta2 - theta3;

    % Output joint angles
    joint_angles = [theta1, theta2, theta3, theta4];
end

function plot_robot_arm(joint_angles, L1, L2, L3, L4)
    % Extract joint angles
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    theta4 = joint_angles(4);

    % Forward kinematics to calculate joint positions
    T01 = dh_matrix(theta1, 0, L1, pi/2);
    T12 = dh_matrix(theta2, L2, 0, 0);
    T23 = dh_matrix(theta3, L3, 0, 0);
    T34 = dh_matrix(theta4, L4, 0, 0);

    % Concatenate matrices without transposing position vectors
    joint_positions = [zeros(1,3); T01(1:3, 4)', {T01 * T12}(1:3, 4)' {T01 * T12 * T23}(1:3, 4)', {T01 * T12 * T23 * T34}(1:3, 4)'];


    % Plot the robot arm
    figure;
    plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), '-o', 'LineWidth', 2);
    hold on;
    grid on;
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    title('Robot Arm Pose');
    axis equal;
    view(3);
end

function T = dh_matrix(theta, a, d, alpha)
    % Calculate DH transformation matrix
    % Inputs:
    %   theta: Joint angle (radians)
    %   a: Link length
    %   d: Joint offset
    %   alpha: Link twist (radians)
    % Output:
    %   T: 4x4 transformation matrix

    T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end



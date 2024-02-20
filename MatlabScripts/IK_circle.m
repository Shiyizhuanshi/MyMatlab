clear

AB = 100;
BC = 100;
CT = 100;

jointA_limit = [-pi, pi];
jointB_limit = [-pi, pi];
jointC_limit = [-pi, pi];

T = [000, 200];




% circle radius
radius = 50;  %radius
center = [200, 100];  % center
num_points = 400;
theta = linspace(0, 2*pi, num_points);
x1 = center(1) + radius * cos(theta);
y1 = center(2) + radius * sin(theta);

x2 = linspace(-200, 200, num_points);
y2 = linspace(200, 200, num_points);

% square side length
side_length = 400;
num_points_per_side = num_points/4;

%left bottom corner
C = [-200, -200];

x3 = [];
y3 = [];

% add bottom
x3 = [x3, linspace(C(1), C(1)+side_length, num_points_per_side)];
y3 = [y3, linspace(C(2), C(2), num_points_per_side)];

% add right side
x3 = [x3, C(1)+side_length * ones(1, num_points_per_side)];
y3 = [y3, linspace(C(2),C(2) + side_length, num_points_per_side)];

% add top
x3 = [x3, linspace(C(1) + side_length, C(1), num_points_per_side)];
y3 = [y3, C(2) + side_length * ones(1, num_points_per_side)];

% add left side
x3 = [x3, linspace(C(1), C(1), num_points_per_side)];
y3 = [y3, linspace(C(2) + side_length, C(2), num_points_per_side)];


default_fig_position = [500, 300, 1200, 1000]; % [left, bottom, width, height]
% Plotting the robot arm
figure('Position', default_fig_position);
hold on;
axis equal;
axis([-300, 300, -300, 300]);
grid on;

    
for i = 1:num_points
    cla;
    % T = [x1(i), y1(i)];
    % T = [x2(i), y2(i)];
    T = [x3(i), y3(i)];
    
    % T = [150,0];
    % plot(x1, y1, 'LineWidth', 2, 'Marker', 'o', 'Color', 'b');
    % plot(x2, y2, 'LineWidth', 2, 'Marker', 'o', 'Color', 'b');
    plot(x3, y3, 'LineWidth', 2, 'Marker', 'o', 'Color', 'b');
    
    % Inverse kinematics
    iteration = 1;
    T_angle_initial_guess = -pi;
    while iteration <= 4000
        
        %get the position of C
        pos_C = [T(1) - CT * cos(T_angle_initial_guess), T(2) - CT * sin(T_angle_initial_guess)];
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
                jointA_angle = atan2(pos_C(2), pos_C(1)) - asin(BC*sin(jointB_angle)/AC);
            else
                jointA_angle = atan2(pos_C(2), pos_C(1)) + asin(BC*sin(jointB_angle)/AC);
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

    
    fprintf('Joint A: %f, Joint B: %f, Joint C: %f\n', jointA_angle, jointB_angle, jointC_angle);
    
    % DH参数（Denevit-Hartenberg参数）
    a = [100, 100, 100];     % 关节长度
    alpha = [0, 0, 0]; % 关节旋转轴的夹角
    d = [0, 0, 0];     % 关节的位移

    % Transformation matrices
    T1 = dh_matrix(a(1), alpha(1), d(1), jointA_angle);
    T2 = dh_matrix(a(2), alpha(2), d(2), jointB_angle);
    T3 = dh_matrix(a(3), alpha(3), d(3), jointC_angle);

    % Calculate link end points
    linkA_end = T1 * [0; 0; 1];
    linkB_end = T1 * T2 * [0; 0; 1];
    linkC_end = T1 * T2 * T3 * [0; 0; 1];

    % fprintf('The linkA_end: %d, %d\n', linkA_end(1), linkA_end(2));
    % fprintf('The linkB_end: %d, %d\n', linkB_end(1), linkB_end(2));
    % fprintf('The linkC_end: %d, %d\n', linkC_end(1), linkC_end(2));
    


    % Plotting link AB
    jointA = [0,0];
    jointB = linkA_end;
    fprintf('The jointB: %d, %d\n', jointB(1), jointB(2));

    plot([jointA(1), jointB(1)], [jointA(2), jointB(2)], 'LineWidth', 2, 'Marker', 'o', 'Color', 'b');

    % Plotting link BC
    jointC = linkB_end;
    plot([jointB(1), jointC(1)], [jointB(2), jointC(2)], 'LineWidth', 2, 'Marker', 'o', 'Color', 'r');

    % Plotting link CT
    endEffector = linkC_end;
    plot([jointC(1), endEffector(1)], [jointC(2), endEffector(2)], 'LineWidth', 2, 'Marker', 'o', 'Color', 'g');

    % Plotting the joints
    plot(jointA(1), jointA(2), 'Marker', 'o', 'Color', 'b');
    plot(jointB(1), jointB(2), 'Marker', 'o', 'Color', 'b');
    plot(jointC(1), jointC(2), 'Marker', 'o', 'Color', 'r');
    plot(endEffector(1), endEffector(2), 'Marker', 'o', 'Color', 'g');

    drawnow;
    pause(0.01);
end

title('Robot Arm Configuration');
xlabel('X-axis');
ylabel('Y-axis');
legend('Link AB', 'Link BC', 'Link CT', 'Joint A', 'Joint B', 'Joint C', 'End Effector');
hold off;









function [jointA_angle, jointB_angle, jointC_angle] = IK(T, T_angle_initial_guess, AB, BC, CT, jointA_limit, jointB_limit, jointC_limit)
    fprintf('Target: %d, %d\n', T(1), T(2));
    iteration = 1;

    while iteration <= 400
        %get the position of C
        pos_C = [T(1) - CT * cos(T_angle_initial_guess), T(2) - CT * sin(T_angle_initial_guess)];
        AC = sqrt((pos_C(1))^2 + (pos_C(2))^2);
        fprintf('pos_C: %d, %d\n', pos_C(1), pos_C(2));
        fprintf('The AC: %d, AB + BC: %d\n', AC, AB + BC);

        % Check AC First    
        if AC > AB + BC
            T_angle_initial_guess = T_angle_initial_guess + deg2rad(1);
            iteration = iteration + 1;
            fprintf('AC is greater than AB + BC in iteration: %d\n', iteration);
            continue;
        else
            % check no joint angle is out of limit
            AC_angle = atan2(pos_C(2), pos_C(1));
            S = AB + BC + CT;
            jointA_angle = asin(S/(AB*AC)) + AC_angle;
            jointB_angle = asin(S/(AB*BC));
            jointC_angle = T_angle_initial_guess-jointA_angle-jointB_angle;
            
            if jointA_angle < jointA_limit(1) || jointA_angle > jointA_limit(2) || ...
                jointB_angle < jointB_limit(1) || jointB_angle > jointB_limit(2) || ...
                jointC_angle < jointC_limit(1) || jointC_angle > jointC_limit(2)
                fprintf('Joint angle out of limit in iteration: %d\n', iteration);
                T_angle_initial_guess = T_angle_initial_guess + deg2rad(1);
                iteration = iteration + 1;
            else
                break;
            end
        end
    end

    fprintf('The jointA_angle: %f, jointB_angle: %f, jointC_angle: %f\n', ...
        (jointA_angle), (jointB_angle), (jointC_angle));
end

function transformationMatrix = dh_matrix(a, alpha, d, theta)
    % 计算DH参数对应的转换矩阵（2D）
    transformationMatrix = [cos(theta), -sin(theta), a*cos(theta);
                            sin(theta), cos(theta), a*sin(theta);
                            0, 0, 1];
end
lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID1                      = 11;            % Dynamixel ID: 1
DXL_ID2                      = 12;
DXL_ID3                      = 13;
DXL_ID4                      = 14;
DXL_GRIP                     = 15; 
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM10';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2047;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
%dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
dxl_goal_position = [DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_j1 = 3400;
MIN_j1 = 600;

MAX_j2 = 3100;
MIN_j2 = 800;

MAX_j3 = 2950;
MIN_j3 = 800;

MAX_j4 = 3350;
MIN_j4 = 966;

MAX_grip = 2620;
MIN_grip = 1000;
%----Grip----%
grip_close=2300;
grip_open=1500;

% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_j1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_j2);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MAX_POS, MAX_j3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MAX_POS, MAX_j4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_MAX_POS, MAX_grip);

% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_j1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_j2);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MIN_POS, MIN_j3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MIN_POS, MIN_j4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_MIN_POS, MIN_grip);

% ---------------------------------- %

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_OPERATING_MODE, 3);


%put into timebased
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, 10, 4);

%profile vel
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 112,2*1500);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 112,2*1500);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 112,2*1500);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 112,2*1500);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, 112,2*1500);


% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

dxl_present_position2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);

startPos = [];

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,3125);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION,grip_open);
pause(2)

%<--------------------------------------------------------------------------------------------------------------------------->



L1 = 77;
L2 = 130;
L3 = 124;
L4 = 126;

thetaOffset = asin(24/130);
default_fig_position = [500, 300, 1200, 1000];
link_colors = {'m-','k-', 'b-', 'g-'};

joint1_limit = [deg2rad(0), deg2rad(360)];
joint2_limit = [deg2rad(100), deg2rad(330)];
joint3_limit = [deg2rad(30), deg2rad(330)];
joint4_limit = [deg2rad(30), deg2rad(330)];

% given desired position and calculate the joint angles

T_3d = [-75, 200, 80];

[joint1_angle, joint2_angle, joint3_angle, joint4_angle] = IK(T_3d, -pi/2, L1, L2, L3, L4, ...
                                                                joint1_limit, joint2_limit, joint3_limit, joint4_limit);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION,grip_open);
pause(1)
moveToAngles(joint1_angle, joint2_angle - thetaOffset, joint3_angle + thetaOffset, joint4_angle, grip_open);
pause(3)

% fprintf('waiting to reach desired position\n');
% pause(10);
% fprintf('plotting the robot arm based on reading\n');
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION,grip_close);
% pause(2)
% theta1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION*0.088);
% theta2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, (ADDR_PRO_PRESENT_POSITION + thetaOffset)*0.088);
% theta3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, (ADDR_PRO_PRESENT_POSITION - thetaOffset)*0.088);
% theta4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION*0.088);

T_3d = [-125, 125, 80];

[joint1_angle, joint2_angle, joint3_angle, joint4_angle] = IK(T_3d, -pi/2, L1, L2, L3, L4, ...
                                                                joint1_limit, joint2_limit, joint3_limit, joint4_limit);

moveToAngles(joint1_angle, joint2_angle - thetaOffset, joint3_angle + thetaOffset, joint4_angle, grip_open);

% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,rad2deg(joint1_angle)/0.088);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,(rad2deg(joint2_angle - thetaOffset))/0.088);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,(rad2deg(joint3_angle + thetaOffset))/0.088);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,rad2deg(joint4_angle)/0.088);

fprintf('waiting to reach desired position\n');
pause(10);
fprintf('plotting the robot arm based on reading\n');

theta1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION*0.088);
theta2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, (ADDR_PRO_PRESENT_POSITION + thetaOffset)*0.088);
theta3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, (ADDR_PRO_PRESENT_POSITION - thetaOffset)*0.088);
theta4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION*0.088);




[theta1, theta2, theta3, theta4] = ServoAnglesToIkAngles(theta1, theta2, theta3, theta4);

% Initialize figure
%{

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

 hold off;


%}


% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


%{
function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ...
    IK(T_3d, end_ang, L12, L23, L34, L45, joint1_limit, joint2_limit, joint3_limit, joint4_limit)
    
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

        %{
        if joint1_angle < joint1_limit(1) || joint1_angle > joint1_limit(2) || ...
            joint2_angle < joint2_limit(1) || joint2_angle > joint2_limit(2) || ...
            joint3_angle < joint3_limit(1) || joint3_angle > joint3_limit(2) || ...
            joint4_angle < joint4_limit(1) || joint4_angle > joint4_limit(2)
            fprintf('Joint angle out of limit\n');
            fprintf('joint1_angle: %f\n', rad2deg(joint1_angle));
            fprintf('joint2_angle: %f\n', rad2deg(joint2_angle));
            fprintf('joint3_angle: %f\n', rad2deg(joint3_angle));
            fprintf('joint4_angle: %f\n', rad2deg(joint4_angle));
            end_ang = end_ang + stepSize;
            continue;
        end
        %}
        solutions_array = [solutions_array; [joint1_angle, joint2_angle, joint3_angle, joint4_angle]];
    end
    if size(solutions_array) == 0
        error('No solution found');
    end
    fprintf('Number of solutions: %d\n', size(solutions_array, 1));
    fprintf('First solution:\ntheta1:%f\ntheta2:%f\ntheta3:%f\ntheta4:%f\n', rad2deg(solutions_array(1,1)), rad2deg(solutions_array(1,2)), rad2deg(solutions_array(1,3)), rad2deg(solutions_array(1,4)));
end


%}


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

% function [joint1_limit_new, joint2_limit_new, joint3_limit_new, joint4_limit_new] = servoLimitesToIkLimits(joint1_limit, joint2_limit, joint3_limit, joint4_limit)
%     joint1_limit_new = [joint1_limit(1), joint1_limit(2)];
%     joint2_limit_new = [3*pi/2 - joint2_limit(1), 3*pi/2 - joint2_limit(2)];
%     joint3_limit_new = [pi/2 - joint3_limit(2), pi/2 - joint3_limit(1)];
%     joint4_limit_new = [pi - joint4_limit(2), pi - joint4_limit(1)];
% end

function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ikAnglesToServoAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
    fprintf('ik_joint1_angle: %f\n', rad2deg(joint1_angle));
    fprintf('ik_joint2_angle: %f\n', rad2deg(joint2_angle));
    fprintf('ik_joint3_angle: %f\n', rad2deg(joint3_angle));
    fprintf('ik_joint4_angle: %f\n', rad2deg(joint4_angle));
    joint2_angle = 3*pi/2 - joint2_angle;
    joint3_angle = -joint3_angle + pi/2 ;
    joint4_angle = pi - joint4_angle;
    fprintf('servo_joint1_angle: %f\n', rad2deg(joint1_angle));
    fprintf('servo_joint2_angle: %f\n', rad2deg(joint2_angle));
    fprintf('servo_joint3_angle: %f\n', rad2deg(joint3_angle));
    fprintf('servo_joint4_angle: %f\n', rad2deg(joint4_angle));
end

function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ServoAnglesToIkAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
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

function moveToAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle, grip_angle)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, rad2deg(joint1_angle)/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, rad2deg(joint2_angle)/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, rad2deg(joint3_angle)/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, rad2deg(joint4_angle)/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, grip_angle);
end
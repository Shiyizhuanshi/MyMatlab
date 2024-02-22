% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

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
DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
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


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

th_offset = asin(24/130)*180/pi;

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

L1 = 43;
L2 = 130;
L3 = 124;
L4 = 126;

% Define Denavit-Hartenberg parameters
a = [0, L2, L3, L4];  % Link lengths
alpha = [pi/2, 0, 0, 0];  % Link twists
d = [77, 0, 0, 0];  % Joint offsets

%link_colors = {'b-', 'g-', 'm-', 'c-'};
link_colors = {'m-','k-', 'b-', 'g-'};

% Number of points for visualization
num_points = 100;

theta1_desire_start = pi;
theta2_desire_start = 0;
theta3_desire_start = 0;
theta4_desire_start = 0;

theta1_desire_end = -pi/2;
theta2_desire_end = pi/2;
theta3_desire_end = -pi/2;
theta4_desire_end = -pi/2;



% Initialize joint angles
theta1 = linspace(theta1_desire_start, theta1_desire_end, num_points);
theta2 = linspace(theta2_desire_start, theta2_desire_end, num_points);
theta3 = linspace(theta3_desire_start, theta3_desire_end, num_points);
theta4 = linspace(theta4_desire_start, theta4_desire_end, num_points);

theta1_d=theta1*180/pi;
theta2_d=theta2*180/pi;
theta3_d=theta3*180/pi;
theta4_d=theta4*180/pi;


default_fig_position = [0, 0, 1200, 1000]; % [left, bottom, width, height]

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
j = 0;
while(j < 1000)

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,theta1_d(1,j+1)/0.088);
    pause(0.0001)
    
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,(180-th_offset)/0.088);
    pause(1)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,(180+th_offset)/0.088);
    pause(1)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,180/0.088);

    joint_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    joint_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    joint_3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PRESENT_POSITION);
    joint_4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    grip = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_PRESENT_POSITION);

    joint_1 = joint_1*0.088;
    joint_2 = (joint_2*0.088)-90-asin(24/130);
    joint_3 = (joint_3*0.088)-90+asin(24/130);
    joint_4 = (joint_4*0.088)+ 180;

    fprintf('Joint 2 Angle (degrees): %.2f\n', joint_2);
    fprintf('Joint 3 Angle (degrees): %.2f\n', joint_3);
    fprintf('Joint 4 Angle (degrees): %.2f\n', joint_4);
    
    j_1_deg = (joint_1*pi)/180;
    j_2_deg = ((joint_2*pi)/180);
    j_3_deg = (joint_3*pi)/180;
    j_4_deg = (joint_4*pi)/180;
    
    fprintf('Joint 1 Angle (degrees), radians: %.2f\n  ,  %.2f\n', joint_1, j_1_deg);
    fprintf('Joint 2 Angle (degrees), radians: %.2f\n  ,  %.2f\n', joint_2, j_2_deg);
    fprintf('Joint 3 Angle (degrees), radians: %.2f\n  ,  %.2f\n', joint_3, j_3_deg);
    fprintf('Joint 4 Angle (degrees), radians: %.2f\n  ,  %.2f\n', joint_4, j_4_deg);
    
    % Find all graphics objects in the current axis
    all_objects = findobj(gca, 'Type', 'line', '-or', 'Type', 'text', '-or', 'Type', 'surface', '-or', 'Type', 'patch', '-or', 'Type', 'hggroup');

    % Exclude the end effector from the list of objects to delete
    end_effector_object = findobj(all_objects, 'Tag', 'EndEffector');
    objects_to_delete = setdiff(all_objects, end_effector_object);

    % Delete objects other than the end effector
    delete(all_objects);

    % Transformation matrices
    T1 = dh_matrix(a(1), alpha(1), d(1), j_1_deg);
    T2 = dh_matrix(a(2), alpha(2), d(2), j_2_deg);
    T3 = dh_matrix(a(3), alpha(3), d(3), j_3_deg);
    T4 = dh_matrix(a(4), alpha(4), d(4), j_4_deg);

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



%{

j = 0;
while (j<1500)
    joint_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    joint_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    joint_3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PRESENT_POSITION);
    joint_4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION);
    grip = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_PRESENT_POSITION);

    j_1_deg = joint_1*0.088;
    j_2_deg = joint_2*0.088;
    j_3_deg = joint_3*0.088;
    j_4_deg = joint_4*0.088;

    fprintf('Joint 1 Angle (degrees): %.2f\n', j_1_deg);
    fprintf('Joint 2 Angle (degrees): %.2f\n', j_2_deg);
    fprintf('Joint 3 Angle (degrees): %.2f\n', j_3_deg);
    fprintf('Joint 4 Angle (degrees): %.2f\n', j_4_deg);
    
    fprintf('[ID:%03d] Position: %03d\n', DXL_GRIP, typecast(uint32(grip), 'int32'));



end
%}


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

function A = dh_matrix(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end



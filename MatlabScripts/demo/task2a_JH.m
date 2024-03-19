
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

DXL_IDs = [DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_GRIP];

BAUDRATE                    = 115200;
DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2047;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

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

for i = 1:length(DXL_IDs)
    % Put actuator into Position Control Mode
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_OPERATING_MODE, 3);
    %put into timebased
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), 10, 4);
    %profile vel
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), 112,0.5*1500);
    % Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
end


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

GRIP_CLOSE = 3000;
GRIP_OPEN = 1700;

% start position
move_robot([-250, 0, 205, 0], port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, 2);
OpenGripper(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, GRIP_CLOSE, COMM_SUCCESS);
%<--------------------------------------------------------------------------------------------------------------------------->
% given desired position and calculate the joint angles

% 1 (-175, 175)
% 2 (-175, -50)
% 3 (0, -175)
% 4 (0, 150)
% 5 (-100, -100)
% 6 (-225, 0)

destinations = {[-72, 212, 120, -pi/2, 1], [-72, 212, 63, -pi/2, 1], GRIP_CLOSE, [-72, 212, 120, -pi/2, 1]...
                [-131, 131, 120, -pi/2, 1], [-131, 137, 65, -pi/2, 1], GRIP_OPEN, [-131, 131, 120, -pi/2, 1]...
                [-225, 0, 120, -pi/2, 1], [-225, 0, 63, -pi/2, 1], GRIP_CLOSE, [-225, 0, 120, -pi/2, 1]...
                [-102, 0, 120, -pi/2, 1], [-102, 1, 49, -pi/2, 1], GRIP_OPEN, [-100, 0, 120, -pi/2, 1]...
                [-150, -150, 120, -pi/2, 1], [-150, -150, 68, -pi/2, 1], GRIP_CLOSE, [-150, -150, 120, -pi/2, 1]...
                [0, -100, 120, -pi/2, 1], [0, -100, 55, -pi/2, 1], GRIP_OPEN, [0, -100, 120, -pi/2, 1]};

destinations = {[-175, 175, 120, 0, 1], [-175, 175, 65, 0, 1], GRIP_CLOSE, [-175, 175, 120, 0, 1]...
                [-104.8, 104.8, 120, -pi/2, 1], [-104.8, 104.8, 65, -pi/2, 1], GRIP_OPEN, [-104.8, 104.8, 120, -pi/2, 1]...
                [-175, -50, 120, -pi/2, 1], [-175, -50, 63, -pi/2, 1], GRIP_CLOSE, [-175, -50, 120, -pi/2, 1]...
                [-225, 0, 120, -pi/2, 1], [-225, 0, 63, -pi/2, 1], GRIP_OPEN, [-225, 0, 120, -pi/2, 1]...
                [0, -175, 120, -pi/2, 1], [0, -175, 55, -pi/2, 1], GRIP_CLOSE, [0, -175, 120, -pi/2, 1]...
                [0, 150, 120, -pi/2, 1], [0, 150, 55, -pi/2, 1], GRIP_OPEN, [0, 150, 120, -pi/2, 1]};

% destinations_test_furthest_point = {[-168, 53, 120, 0, 1], [-168, 53, 65, 0, 1], GRIP_CLOSE, [-168, 53, 120, 0, 1]};

for i = 1:length(destinations)
    if length(destinations{i}) == 5
        move_robot(destinations{i}(1:4), port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, destinations{i}(5));
    elseif destinations{i} == GRIP_CLOSE
        OpenGripper(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, GRIP_OPEN, COMM_SUCCESS);
        %write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, (GRIP_CLOSE));
    else
        CloseGripper(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, GRIP_CLOSE, COMM_SUCCESS);
        %write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, (GRIP_OPEN));
    end
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);



function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ...
    IK(T_3d, end_ang, L12, L23, L34, L45)
    thetaOffset = asin(24/130);
    iteration = 0;
    solutions_array = [];
    stepSize = deg2rad(0.1);

    while iteration < 4000 && size(solutions_array, 1) < 3
        iteration = iteration + 1;
        % fprintf('Iteration: %d\n', iteration);
        % fprintf('end_ang: %f\n', rad2deg(end_ang));
        joint1_angle = atan2(T_3d(2), T_3d(1));
        if (joint1_angle < 0)
            joint1_angle = joint1_angle + 2* pi;
        end
        % if (joint1_angle < pi/2)
        %     joint1_angle = joint1_angle + pi;
        % end
        T_2d = [sqrt(T_3d(1)^2 + T_3d(2)^2), T_3d(3)-L12]; 
        pos_4 = [T_2d(1) - cos(end_ang)*L45, T_2d(2) - sin(end_ang)*L45];
        L24_angle = atan2(pos_4(2), pos_4(1));
        L24 = sqrt(pos_4(1)^2 + pos_4(2)^2);
        % fprintf('pos_4: %f, %f\n', pos_4(1), pos_4(2));
        % fprintf('L24_angle: %f\n', rad2deg(L24_angle));
        % fprintf('L24: %f\n', L24); 
        if L24 > L23 + L34
            % fprintf('L24 is too long\n');
            end_ang = end_ang + stepSize;
            continue;
        end
        joint3_angle = -acos((L24^2 - L23^2 - L34^2)/(2*L23*L34));
        joint2_angle = L24_angle + acos((L23^2 + L24^2 - L34^2)/(2*L23*L24));
        joint4_angle = end_ang - joint2_angle - joint3_angle;

        [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ikAnglesToServoAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle);
        
        solutions_array = [solutions_array; [joint1_angle, joint2_angle, joint3_angle, joint4_angle]];
    end
    if size(solutions_array) == 0
        error('No solution found');
    end
    % fprintf('Number of solutions: %d\n', size(solutions_array, 1));
    % fprintf('First solution:\ntheta1:%f\ntheta2:%f\ntheta3:%f\ntheta4:%f\n', ...
    %     rad2deg(solutions_array(1,1)), rad2deg(solutions_array(1,2) - thetaOffset), rad2deg(solutions_array(1,3) + thetaOffset), rad2deg(solutions_array(1,4)));
end


function [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = ikAnglesToServoAngles(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
    % fprintf('ik_joint1_angle: %f\n', rad2deg(joint1_angle));
    % fprintf('ik_joint2_angle: %f\n', rad2deg(joint2_angle));
    % fprintf('ik_joint3_angle: %f\n', rad2deg(joint3_angle));
    % fprintf('ik_joint4_angle: %f\n', rad2deg(joint4_angle));
    joint2_angle = 3*pi/2 - joint2_angle;
    joint3_angle = -joint3_angle + pi/2 ;
    joint4_angle = pi - joint4_angle;
    % fprintf('servo_joint1_angle: %f\n', rad2deg(joint1_angle));
    % fprintf('servo_joint2_angle: %f\n', rad2deg(joint2_angle));
    % fprintf('servo_joint3_angle: %f\n', rad2deg(joint3_angle));
    % fprintf('servo_joint4_angle: %f\n', rad2deg(joint4_angle));
end


function move_robot(destination_coordinates, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,moving_time)
    L1 = 77;
    L2 = 130;
    L3 = 124;
    L4 = 126;
    DXL_ID1 = 11;
    DXL_ID2 = 12;
    DXL_ID3 = 13;
    DXL_ID4 = 14;
    thetaOffset = asin(24/130);

    end_ang = destination_coordinates(4);
    destination_coordinates = [destination_coordinates(1), destination_coordinates(2), destination_coordinates(3)];

    disp("Moving to destination:");
    disp(destination_coordinates);
    disp("end angle:");
    disp(end_ang);

    [joint1_angle, joint2_angle, joint3_angle, joint4_angle] = IK(destination_coordinates, end_ang, L1, L2, L3, L4);

    disp("Joint angles:");
    disp([rad2deg(joint1_angle), rad2deg(joint2_angle), rad2deg(joint3_angle), rad2deg(joint4_angle)]);

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,rad2deg(joint1_angle)/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,(rad2deg(joint2_angle - thetaOffset))/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,(rad2deg(joint3_angle + thetaOffset))/0.088);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,rad2deg(joint4_angle)/0.088);
    pause(moving_time);
end


% function MoveGripper(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, GRIP_OPEN_POS, COMM_SUCCESS)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, (GRIP_OPEN_POS));
%     dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
%     dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
%     if dxl_comm_result ~= COMM_SUCCESS
%         fprintf('%s\n\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%     elseif dxl_error ~= 0
%         fprintf('%s\n\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%     end
%     pause(2);
% end

function OpenGripper(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, GRIP_OPEN_POS, COMM_SUCCESS)
    pause(0.5);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(GRIP_OPEN_POS), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    pause(1);
end

function CloseGripper(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, GRIP_CLOSE_POS, COMM_SUCCESS)
    pause(0.5);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(GRIP_CLOSE_POS), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    pause(1);
end
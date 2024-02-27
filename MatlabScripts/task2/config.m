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

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION,2045);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION,3125);
pause(1)

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION,1200);
pause(1)

save('config_variables.mat');

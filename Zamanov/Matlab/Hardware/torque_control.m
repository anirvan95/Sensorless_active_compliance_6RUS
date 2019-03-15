%Interfacing with the Hardware using Dynamixel SDK

clc;
clear all;
close all;


%Load Dynamixel Library
lib_name = 'libdxl_x64_c';
% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       	= 24;                               % Control table address is different in Dynamixel model
ADDR_MX_TORQUE_CONTROL_ENABLE	= 70;
ADDR_MX_GOAL_POSITION           = 30;
ADDR_MX_PRESENT_POSITION        = 36;
ADDR_MX_MOVING_VELOCITY         = 32;
ADDR_MX_PRESENT_VELOCITY        = 38;
ADDR_MX_PRESENT_CURRENT         = 68;
ADDR_MX_PRESENT_LOAD            = 40;
ADDR_MX_GOAL_TORQUE              = 71;                            	
ADDR_MX_GOAL_ACCELERATION       = 73;
IDs                             = 4;
dxl_goal_velocity = speed_to_dxl(30);
% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel
LEN_MX_GOAL_POSITION = 2;
LEN_MX_MOVING_VELOCITY = 2;
LEN_MX_GOAL_TORQUE = 2;

%% Initialization

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
DEVICENAME = '/dev/ttyUSB0';
BAUDRATE = 1000000;
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end
% 
% Enable Dynamixel Torques

for i = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(i), ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    check_error(port_num);
end

% Provide Goal Torque
group_tor = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_TORQUE, LEN_MX_GOAL_TORQUE);
for index = 1:length(IDs)
    dxl_addparam_result = groupSyncWriteAddParam(group_tor, IDs(index), 2040, LEN_MX_GOAL_TORQUE);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', IDs(index));
        break;
    end
end

closePort(port_num);
% plot(time_vect,dxl_present_current_vect(:,1)*1000,'r')
% a = 1;
% b = [1/5,1/5,1/5,1/5,1/5];
% y = filter(b,a,dxl_present_current_vect(:,1)*1000);
% hold on;
% plot(time_vect, y);
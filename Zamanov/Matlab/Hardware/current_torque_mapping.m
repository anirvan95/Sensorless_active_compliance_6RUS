%Interfacing with the Hardware using Dynamixel SDK

clc;
clear all;
close all;

prev_t = 0;
zero_corr = -14.73;

%Load Dynamixel Library
lib_name = 'libdxl_x64_c';
% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       	= 24;                               % Control table address is different in Dynamixel model
ADDR_MX_TORQUE_CONTROL_ENABLE	= 70;
ADDR_MX_GOALTORQUE              = 71;
ADDR_MX_GOAL_ACCELERATION       = 73;
ADDR_MX_GOAL_POSITION           = 30;
ADDR_MX_PRESENT_POSITION        = 36;
ADDR_MX_MOVING_VELOCITY         = 32;
ADDR_MX_PRESENT_VELOCITY        = 38;
ADDR_MX_PRESENT_CURRENT         = 68;
ADDR_MX_PRESENT_LOAD            = 40;
IDs                             = 6;
dxl_goal_velocity = speed_to_dxl(30);
% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel
LEN_MX_GOAL_POSITION = 2;
LEN_MX_MOVING_VELOCITY = 2;


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


% Provide  Moving Velocities 
group_vel = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_MOVING_VELOCITY, LEN_MX_MOVING_VELOCITY);
for index = 1:length(IDs)
    dxl_addparam_result = groupSyncWriteAddParam(group_vel, IDs(index), dxl_goal_velocity, LEN_MX_MOVING_VELOCITY);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', IDs(index));
        break;
    end
end
% Syncwrite goal position
groupSyncWriteTxPacket(group_vel);

%% Providing position commands to the Dynamixel motors
% Initialize Groupsyncwrite instance for postion and velocity
group_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

Tp = 100;

dxl_present_current_vect = [];
dxl_present_load_vect = [];
time_vect = [];
dxl_present_current = zeros(length(IDs),1);
dxl_present_load = zeros(length(IDs),1);
angle_vect = [];
tic
t = 0;
%%Inverse Kinematics
while(t<Tp)
    t = toc;
    %Provide Desired Trajectory
    y = 0.120*(abs(cos(t)));
    %theta = acos(y/0.120);
    if t>130 && t<135
        theta = 1.047;
    else
        theta = 0;
    end
    
    angle_vect = [angle_vect;rad2deg(theta)];
    for index = 1:length(IDs)
        dxl_addparam_result = groupSyncWriteAddParam(group_pos, IDs(index), degree_to_dxl(rad2deg(theta)+zero_corr(1)), LEN_MX_GOAL_POSITION);
        %dxl_addparam_result = groupSyncWriteAddParam(group_pos, IDs(index), degree_to_dxl(0), LEN_MX_GOAL_POSITION);
        
        if dxl_addparam_result ~= true
            fprintf('[ID:%03d] groupSyncWrite addparam failed', IDs(index));
            break;
        end
    end
    % Syncwrite goal position
    groupSyncWriteTxPacket(group_pos);
    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(group_pos);
    
    
    %Get Current and Load Feedback
    for index = 1:length(IDs)
        dxl_present_current(index) = read2ByteTxRx(port_num, PROTOCOL_VERSION, IDs(index), ADDR_MX_PRESENT_CURRENT);
        present_current(index) = 4.5*(dxl_present_current(index) - 2048)/1000;
        dxl_comm_result = check_error(port_num);
        dxl_present_load(index) = read2ByteTxRx(port_num, PROTOCOL_VERSION, IDs(index), ADDR_MX_PRESENT_LOAD);
        if(dxl_present_load(index) > 1024)
            dxl_present_load(index) = -(dxl_present_load(index) - 1024)*0.1*5;
        else
            dxl_present_load(index) = dxl_present_load(index)*0.1*5;
        end
        dxl_comm_result = check_error(port_num);
    end
    if abs(present_current) < 5
        dxl_present_current_vect = [dxl_present_current_vect;present_current'];
        dxl_present_load_vect = [dxl_present_load_vect;dxl_present_load'];
        time_vect = [time_vect;t];
        dxl_present_load_vect
        subplot(2,1,1), plot(time_vect,abs(dxl_present_current_vect),'r')
        axis([0 Tp 0 0.5])
        subplot(2,1,2), plot(time_vect,abs(dxl_present_load_vect), 'b')
        axis([0 100 0 100])
        drawnow;
    end
end

closePort(port_num);
% plot(time_vect,dxl_present_current_vect(:,1)*1000,'r')
% a = 1;
% b = [1/5,1/5,1/5,1/5,1/5];
% y = filter(b,a,dxl_present_current_vect(:,1)*1000);
% hold on;
% plot(time_vect, y);
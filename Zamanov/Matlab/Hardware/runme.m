%Interfacing with the Hardware using Dynamixel SDK

clc;
clear all;
close all;
[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
[ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = trajectory();

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
IDs                             = [14,6];
dxl_goal_velocity = speed_to_dxl(50);
% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel
LEN_MX_GOAL_POSITION = 2;
LEN_MX_MOVING_VELOCITY = 2;


dxl_present_current = zeros(length(IDs),1);
dxl_present_load = zeros(length(IDs),1);

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

% Enable Dynamixel Torques

for i = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(i), ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = check_error(port_num);
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


%%Inverse Kinematics
for i=1:length(ex)
	[top_matrix_regen, end_effector_regen, top_matrix_regen_check] = end_effector_regeneration(r_p, theta_p);
	[Top_matrix] = end_effector(ex(i), ey(i), ez(i), roll(i), pitch(i), yaw(i), theta_p, r_p);
	[theta_one, theta_two, theta_three, theta_14, theta_15, theta_16, M_matrix, K_matrix] = inv_kin(Top_matrix, Base_matrix, l1, L2, ex(i), ey(i), ez(i), r_p, top_matrix_regen);
	
    
    %Provide Desired Trajectory
    for index = 1:length(IDs)
        dxl_addparam_result = groupSyncWriteAddParam(group_pos, IDs(index), degree_to_dxl(-1^(index+1)*rad2deg(theta_one(index))), LEN_MX_GOAL_POSITION);
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
        dxl_comm_result = check_error(port_num);
        dxl_present_load(index) = read2ByteTxRx(port_num, PROTOCOL_VERSION, IDs(index), ADDR_MX_PRESENT_LOAD);
        dxl_comm_result = check_error(port_num);
    end
    
    
    
% 	plot3([Base_matrix(1,:), Base_matrix(1,1)], [Base_matrix(2,:), Base_matrix(2,1)], [Base_matrix(3,:), Base_matrix(3,1)], 'Linewidth', 3);
% 	hold on;
% 	
% 	%%Plotting the end_effector
% 	plot3([Top_matrix(1,:), Top_matrix(1,1)], [Top_matrix(2,:), Top_matrix(2,1)], [Top_matrix(3,:), Top_matrix(3,1)],'k', 'Linewidth', 2);
% 	
% 	
% 	%%Plotting all links
% 	for loop_v = 1:6
% 	plot3([Base_matrix(1,loop_v), M_matrix(1,loop_v), K_matrix(1,loop_v)], [Base_matrix(2,loop_v), M_matrix(2,loop_v), K_matrix(2,loop_v)], [Base_matrix(3,loop_v), M_matrix(3,loop_v), K_matrix(3,loop_v)], 'r', 'Linewidth', 3);
% 	end
% 	
% 	grid on;
% 	hold off;
% 	
% 	axis([-0.80 0.80 -0.80 0.80 -0.80 0.80]);
% 	view(90,0);
% 	pause(0.1)


end

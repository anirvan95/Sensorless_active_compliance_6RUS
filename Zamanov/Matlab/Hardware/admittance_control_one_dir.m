%Interfacing with the Hardware using Dynamixel SDK

clc;
clear all;
close all;

prev_t = 0;

[base_length, top_length, half_angle, rem_angle, Base_matrix, l1, L2, theta_p, r_p] = parameters();
%[ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = trajectory();

zero_corr = [-28.35+24.75, 16.22-24.75, -39.6+24.75, 26.77-24.75, -33.1+24.75, 14.64-24.75];

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
IDs                             = [1,2,3,4,5,6];
dxl_goal_velocity = speed_to_dxl(10);
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
DEVICENAME = '/dev/ttyUSB1';
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


% Provide Moving Velocities 
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

first_flag = 0;

Tp = 30;
ti = 5;
ta = ti;   %time when force is applied
Fx_ext = -0.15;
force_applied_flag = 0;
force_removed_flag = 0;
first_case_force_applied = 0;
first_case_force_removed = 0;
dxl_present_current_vect = [];
dxl_present_load_vect = [];
time_vect = [];
dxl_present_current = zeros(length(IDs),1);
dxl_present_load = zeros(length(IDs),1);
step = 0;
h0 = -0.339207;
h1 = -0.5262528;
h2 = 0.5262528;
h3 = 0.3392073;
hpf_value_vect = [];
hpf_value = 0;
tic
t = 0;
%%Inverse Kinematics
while(t<Tp)
    
    t
    %[ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = kintrajectory(ta,tr,t,Fx_ext,force_applied_flag,force_removed_flag);
	%% Trajectory
    if(t<ti)
        disp('Following desired traj')
        xdes = 0.2215;
        ydes = 0.0*cos(4*t);
        zdes = 0.0*sin(4*t);
        rolldes = 0; %Rotation about x-axis in radians
        pitchdes = 0; %Rotation about y-axis in radians
        yawdes = 0; %Rotation about z-axis in radians
        %first derivative
        xdotdes = 0;
        ydotdes = 0;
        zdotdes = 0;
        rolldotdes = 0; %Rotation about x-axis in radians/sec
        pitchdotdes = 0; %Rotation about y-axis in radians
        yawdotdes = 0; %Rotation about z-axis in radians
        %second derivative
        xddotdes = 0;
        yddotdes = 0;
        zddotdes = 0;
        rollddotdes = 0;
        pitchddotdes = 0;
        yawddotdes = 0;
    %%Trajectory when force is applied
    elseif(t>ti)
        hpf_value = h0*dxl_present_current_vect(step) + h1*dxl_present_current_vect(step-1) + h2*dxl_present_current_vect(step-2) + h3*dxl_present_current_vect(step-3);
        
        dxl_present_current_vect(step) = hpf_value;
        force_applied_flag
        force_removed_flag
        abs(hpf_value)
        if (force_applied_flag == 0 && force_removed_flag ==0)
            if(abs(hpf_value)>0.017)
                disp('Force applied')
                force_applied_flag = 1;
                
            else
                disp('No Force applied')
            end
        end
        if(force_applied_flag == 1)
            disp('Force applied following admittance')
            ta
            if (abs(hpf_value)<0.0065)
                force_removed_flag = 1;
                force_applied_flag = 0;
                first_case_force_applied = 0; 
                tr = t;
            else
                if (first_case_force_applied == 0)
                    ta = t;
                    x0 = 0.2215;
                    xe = 0.2215;
                    x0dot = 0;
                    xedot = 0;
                    xeddot = 0;
                    first_case_force_applied = 1;
                end
               
                %%X-direction
                Dd = 10;
                Kd = 5;
                Md = 1;
                xdes = (Fx_ext + Dd*xedot + Kd*xe + Md*xeddot)/Kd - (exp(-((Dd - (Dd^2 - 4*Kd*Md)^(1/2))*(t - ta))/(2*Md))*(Fx_ext + Dd*xedot + Md*xeddot + xedot*(Dd^2 - 4*Kd*Md)^(1/2) - Kd*(x0 - xe) + (Dd*Fx_ext)/(Dd^2 - 4*Kd*Md)^(1/2) - (Kd*Md*(2*x0dot - 4*xedot))/(Dd^2 - 4*Kd*Md)^(1/2) + (Dd*Md*xeddot)/(Dd^2 - 4*Kd*Md)^(1/2) - (Dd*Kd*(x0 - xe))/(Dd^2 - 4*Kd*Md)^(1/2)))/(2*Kd) - (exp(-((t - ta)*(Dd + (Dd^2 - 4*Kd*Md)^(1/2)))/(2*Md))*(Fx_ext + Dd*xedot + Md*xeddot - xedot*(Dd^2 - 4*Kd*Md)^(1/2) - Kd*(x0 - xe) - (Dd*Fx_ext)/(Dd^2 - 4*Kd*Md)^(1/2) + (Kd*Md*(2*x0dot - 4*xedot))/(Dd^2 - 4*Kd*Md)^(1/2) - (Dd*Md*xeddot)/(Dd^2 - 4*Kd*Md)^(1/2) + (Dd*Kd*(x0 - xe))/(Dd^2 - 4*Kd*Md)^(1/2)))/(2*Kd);
                xdotdes = 0;
            end
        %%Trajectory when force is removed
        elseif(force_removed_flag == 1 && force_applied_flag ==0)
            disp('Force removed following admittance')
            if(t - tr > 3)
                force_applied_flag = 0;
                force_removed_flag = 0;
                first_case_force_applied = 0;
                first_case_force_removed = 0;
                dxl_present_current_vect = [];
                dxl_present_load_vect = [];
                time_vect = [];
                dxl_present_current = zeros(length(IDs),1);
                dxl_present_load = zeros(length(IDs),1);
                step = 0;
                force_removed_flag = 0;
                first_case_force_removed = 0;
                ti = 1;
                Fx_ext = -0.15;
                tic
            else
                if(first_case_force_removed ==0)
                    x0 = xdes;
                    x0dot = xdotdes;
                    first_case_force_removed = 1;
                end
                %%X-direction
                Fx_ext = 0;
                Dd = 5;
                Kd = 5;
                Md = 1;
                xdes = (Fx_ext + Dd*xedot + Kd*xe + Md*xeddot)/Kd - (exp(-((Dd - (Dd^2 - 4*Kd*Md)^(1/2))*(t - tr))/(2*Md))*(Fx_ext + Dd*xedot + Md*xeddot + xedot*(Dd^2 - 4*Kd*Md)^(1/2) - Kd*(x0 - xe) + (Dd*Fx_ext)/(Dd^2 - 4*Kd*Md)^(1/2) - (Kd*Md*(2*x0dot - 4*xedot))/(Dd^2 - 4*Kd*Md)^(1/2) + (Dd*Md*xeddot)/(Dd^2 - 4*Kd*Md)^(1/2) - (Dd*Kd*(x0 - xe))/(Dd^2 - 4*Kd*Md)^(1/2)))/(2*Kd) - (exp(-((t - tr)*(Dd + (Dd^2 - 4*Kd*Md)^(1/2)))/(2*Md))*(Fx_ext + Dd*xedot + Md*xeddot - xedot*(Dd^2 - 4*Kd*Md)^(1/2) - Kd*(x0 - xe) - (Dd*Fx_ext)/(Dd^2 - 4*Kd*Md)^(1/2) + (Kd*Md*(2*x0dot - 4*xedot))/(Dd^2 - 4*Kd*Md)^(1/2) - (Dd*Md*xeddot)/(Dd^2 - 4*Kd*Md)^(1/2) + (Dd*Kd*(x0 - xe))/(Dd^2 - 4*Kd*Md)^(1/2)))/(2*Kd);
                xdotdes = 0;
            end
            
        end
    end
    ex = xdes;
    ey = ydes;
    ez = zdes;
    roll = rolldes;
    pitch = pitchdes;
    yaw = yawdes;
    exdot = xdotdes;
    eydot = ydotdes;
    ezdot = zdotdes;
    rolldot = rolldotdes;
    pitchdot = pitchdotdes;
    yawdot = yawdotdes;

    %% Passing Trajectory
    step = step+1;
    [top_matrix_regen, end_effector_regen, top_matrix_regen_check] = end_effector_regeneration(r_p, theta_p);
	[Top_matrix] = end_effector(ex, ey, ez, roll, pitch, yaw, theta_p, r_p);
	[theta_one, theta_two, theta_three, theta_14, theta_15, theta_16, M_matrix, K_matrix] = inv_kin(Top_matrix, Base_matrix, l1, L2, ex, ey, ez, r_p, top_matrix_regen);

    %Provide Desired Trajectory
    for index = 1:length(IDs)
        dxl_addparam_result = groupSyncWriteAddParam(group_pos, IDs(index), degree_to_dxl((-1)^(index+1)*rad2deg(theta_one(index))+zero_corr(index)), LEN_MX_GOAL_POSITION);
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
        dxl_comm_result = check_error(port_num);
    end
    dxl_present_current_vect = [dxl_present_current_vect;sum(abs(present_current))/6];
    dxl_present_load_vect = [dxl_present_load_vect;dxl_present_current'];
    time_vect = [time_vect;t];
    if first_flag == 0
        prev_t = t;
        %pause(1.5)
        first_flag = 1;
    end
    hpf_value_vect = [hpf_value_vect;abs(hpf_value)];
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
% 	view(0,0);
% 	pause(0.1)

    t = toc;
end

closePort(port_num);
% plot(time_vect,dxl_present_current_vect(:,1),'r')
% hold on;
% plot(time_vect,dxl_present_current_vect(:,2),'g')
% plot(time_vect,dxl_present_current_vect(:,3),'b')
% plot(time_vect,dxl_present_current_vect(:,4),'k')
% plot(time_vect,dxl_present_current_vect(:,5),'c')
% plot(time_vect,dxl_present_current_vect(:,6),'m')


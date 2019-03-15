%%Trajectory for the stewart end-effector

function [ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = kintrajectory()
ex=[];
ey=[];
ez=[];
roll=[];
pitch=[];
yaw=[];
exdot = [];
eydot = [];
ezdot = [];
rolldot = [];
pitchdot = [];
yawdot = [];
for i = 1:20
    
    
    ex = [ex;0.25];
    ey = [ey;0.04*sin(i)];
    ez = [ez;0.04*cos(i)];
    roll = [roll;0]; %Rotation about x-axis in radians
    pitch = [pitch;0]; %Rotation about y-axis in radians
    yaw = [yaw;0]; %Rotation about z-axis in radians
    
    exdot = [exdot;0];
    eydot = [eydot;0.04*cos(i)];
    ezdot = [ezdot;-0.04*sin(i)];
    rolldot = [rolldot;0]; %Rotation about x-axis in radians
    pitchdot = [pitchdot;0]; %Rotation about y-axis in radians
    yawdot = [yawdot;0]; %Rotation about z-axis in radians
    
    %
    % 		if unit ==2
    %     ex = [ex;0.50];
    %     ey = [ey;0.12*sin(i)];
    %     ez = [ez;0.12*cos(i)];
    %     roll = [roll;0]; %Rotation about x-axis in radians
    %     pitch = [pitch;0]; %Rotation about y-axis in radians
    %     yaw = [yaw;0]; %Rotation about z-axis in radians
    %
    % 	exdot = [exdot;0];
    %     eydot = [eydot;0.12*cos(i)];
    %     ezdot = [ezdot;-0.12*sin(i)];
    %     rolldot = [rolldot;0]; %Rotation about x-axis in radians
    %     pitchdot = [pitchdot;0]; %Rotation about y-axis in radians
    %     yawdot = [yawdot;0]; %Rotation about z-axis in radians
    % 	end
    
end

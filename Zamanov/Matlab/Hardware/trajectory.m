 %%Trajectory for the stewart end-effector

function [ex, ey, ez, roll, pitch, yaw, exdot, eydot, ezdot, rolldot, pitchdot, yawdot] = trajectory()
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
for t = 1:200
	
    ex = [ex;0.210397];
    ey = [ey;0.025*sin(t)];
    ez = [ez;0.025*cos(t)];
    roll = [roll;0]; %Rotation about x-axis in radians
    pitch = [pitch;0]; %Rotation about y-axis in radians
    yaw = [yaw;0]; %Rotation about z-axis in radians

	exdot = [exdot;0];
    eydot = [eydot;0.0*cos(t)];
    ezdot = [ezdot;-0.0*sin(t)];
    rolldot = [rolldot;0]; %Rotation about x-axis in radians
    pitchdot = [pitchdot;0]; %Rotation about y-axis in radians
    yawdot = [yawdot;0]; %Rotation about z-axis in radians
	end
	
		
end

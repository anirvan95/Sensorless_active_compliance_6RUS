import pypot.dynamixel
import numpy as np
import itertools
import time
import math

def main():

    # Initialization of the Dynamixel Motors
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    port = ports[0]
    print("connecting on the first available port:  ports[0]")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    print("dxl_io:"), dxl_io
    ids = dxl_io.scan([1, 2, 3, 5, 6])
    print("Found ids:"), ids
    print("Present motor position"), dxl_io.get_present_position(ids)
    #setting speed and position to be inital  values
    #speed = dict(zip(ids, itertools.repeat(50)))
    #dxl_io.set_moving_speed(speed)
    #dxl_io.set_goal_position({2: (pos[0][0]), 6: (pos[1][0]), 5: (pos[2][0]), 3: (pos[3][0]), 1: (pos[4][0])})
    print("Setting up")
    #time.sleep(2)
    zero_corr = [0.57, -22.46, 5.85, -5.67, 7.08, -6.37]
    firstflag = 0

    #Define 6RUS design parameter
    base_length = float(30) #Length of bottom platform
    top_length = float(15) #Length of top platform
    l1 = float(12) #Length of link1
    l2 = float(23) #Length of link2
    theta_p = 40*math.pi/180;
    r_p = top_length/(2*math.sin(((2*math.pi/3)-theta_p)/2));
    d = 2*r_p*math.sin(theta_p/2);
    r_b = 2*r_p;
    rem_angle = (2*math.pi/3) - 2*math.asin(0.5*math.sin(theta_p/2)); #Please refer Ikin_convention pic
    half_angle = (2*math.pi/3) - rem_angle; #Please refer Ikin_convention pic
    base_length = 2*r_b*math.sin(rem_angle/2);
    half_angle = math.pi/6 #Please refer Ikin_convention pic
    rem_angle = math.pi/2 #Please refer Ikin_convention pic

    #Define Base Platform
    #Vertex 1 of base platform
    b1 = np.array([[float(0)],[-float(base_length)/(2*math.tan(rem_angle/2))],[-float(base_length)/2]])
    #Vertex 2 of base platform
    rotangle = rem_angle
    Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
    b2 =  Rx_clock.dot(b1)
    #Defining the rest vertices of base platform
    rotangle = rem_angle+half_angle
    Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
    b3 = Rx_clock.dot(b1)
    b4 = Rx_clock.dot(b2)
    b5 = Rx_clock.dot(b3)
    b6 = Rx_clock.dot(b4)
    
    #b1 to b6 are column vectors
    Base_matrix = np.concatenate((b1,b2,b3,b4,b5,b6), axis=1)
    #print Base_matrix
    #t0 is the epoch of the time
    #t1 is the next instant.So, the program runs till t-t0 becomes greater than time specified
    t0 = time.time()
    tr = 0
    increment = 0.01
    print("t0:", t0)
    while tr - t0 <= 20:
        tr = time.time()
        #Define Trajectory
        dt = tr-t0
        print("TIme : ",dt)
        ex = float(20) 
        ey = float(1.5)*math.cos(dt)
        ez = float(1.5)*math.sin(dt)
        roll = 0
        pitch = 0
        yaw = 0
        translate = np.array([[ex],[ey],[ez]])
        R_roll = np.array([[math.cos(roll), -math.sin(roll), 0],[math.sin(roll), math.cos(roll), 0],[0, 0, 1]])
        R_yaw = np.array([[1, 0, 0],[0, math.cos(yaw),-math.sin(yaw)],[0, math.sin(yaw), math.cos(yaw)]])
        R_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)],[0, 1, 0],[-math.sin(pitch), 0, math.cos(pitch)]])

        #Defining the top platform 
        t1 = np.array([[0],[-r_p*math.cos(((2*math.pi/3) - theta_p)/2)],[-r_p*math.sin(((2*math.pi/3) - theta_p)/2)]])
        #Vertex 2 of top platform
        rotangle = (2*math.pi/3) - theta_p
        Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
        t2 =  Rx_clock.dot(t1)
        #Defining the rest vertices of top platform
        rotangle = 2*math.pi/3
        Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
        t3 = Rx_clock.dot(t1)
        t4 = Rx_clock.dot(t2)
        t5 = Rx_clock.dot(t3)
        t6 = Rx_clock.dot(t4)
        t1f = R_roll.dot(R_pitch).dot(R_yaw).dot(t1) + translate 
        t2f = R_roll.dot(R_pitch).dot(R_yaw).dot(t2) + translate
        t3f = R_roll.dot(R_pitch).dot(R_yaw).dot(t3) + translate
        t4f = R_roll.dot(R_pitch).dot(R_yaw).dot(t4) + translate
        t5f = R_roll.dot(R_pitch).dot(R_yaw).dot(t5) + translate
        t6f = R_roll.dot(R_pitch).dot(R_yaw).dot(t6) + translate

        Top_matrix = np.concatenate((t1f,t2f,t3f,t4f,t5f,t6f), axis=1)
        rangle_vectclock = np.array([math.pi/3, -math.pi/3, -math.pi/3, math.pi, math.pi, math.pi/3])
        #print("Rangle_val: ",rangle_vectclock)
        th = []
        for i in range(0,6):
            Rx_clock = np.array([[1, 0, 0],[0, math.cos(rangle_vectclock[i]), math.sin(rangle_vectclock[i])],[0, -math.sin(rangle_vectclock[i]), math.cos(rangle_vectclock[i])]])
            T = Rx_clock.dot(Top_matrix[:,i]-Base_matrix[:,i])
            th3 = -math.asin(T[2]/l2)
            pl2 = l2*math.cos(th3)
            th2 = math.acos((T[0]*T[0] + T[1]*T[1] - l1*l1 - pl2*pl2)/(2*l1*pl2))
            r = math.sqrt(l1*l1 + pl2*pl2 + 2*l1*pl2*math.cos(th2))
            phi = math.atan((l1 + pl2*math.cos(th2))/(pl2*math.sin(th2)))
            th1 = math.asin(T[0]/r)-phi
            th.append(90-(th1*180/math.pi))
        #print(th)
        speed = dict(zip(ids, itertools.repeat(20)))
        dxl_io.set_moving_speed(speed)
        dxl_io.set_goal_position({1: (-th[0]+zero_corr[0]), 2: (th[1]+zero_corr[1]), 3: (-th[2]+zero_corr[2]), 4: (th[3]+zero_corr[3]), 5: (-th[4]+zero_corr[4]), 6: (th[5]+zero_corr[5])})
        if(firstflag == 0):
            time.sleep(5)
            firstflag = 1


if __name__ == "__main__":
    main()

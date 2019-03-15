import itertools
import numpy
import time
import pypot.dynamixel

zero_corr = [0.57, -22.46, 5.85, -5.67, 7.08, -6.37]
#th_abs = [-60, 60, -60, 60, -60, 60]
#theta_req1= [-40,40,-40, 40, -40, 40]
#th_abs = [31.78, -20.09, 3.82, 136.92, -0.75, -169.71]
th_abs = [-56.75, 34.33, -51.03, 41.71, -41.01, 51.91]

if __name__ == '__main__':
    
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    port = ports[0]
    print("connecting on the first available port:  ports[0]")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    print("dxl_io:"), dxl_io
    ids = dxl_io.scan([1, 2, 3, 4, 5, 6])
    print("Found ids:"), ids
    print("Present motor position"), dxl_io.get_present_position(ids)
    speed = dict(zip(ids, itertools.repeat(10)))
    dxl_io.set_moving_speed(speed)
    #dxl_io.set_goal_position({1: (theta_req1[0]+zero_corr[0]), 2: (theta_req1[1]+zero_corr[1]), 3: (theta_req1[2]+zero_corr[2]), 4: (theta_req1[3]+zero_corr[3]), 5: (theta_req1[4]+zero_corr[4]), 6: (theta_req1[5]+zero_corr[5])})
    dxl_io.set_goal_position({1: th_abs[0], 2: th_abs[1], 3: th_abs[2], 4: th_abs[3], 5: th_abs[4], 6: th_abs[5]})
    #tt = dxl_io.get_control_table([1])
    #print("Control mode\n", tt[0]['present current'])
    

import itertools
import numpy
import time
import pypot.dynamixel

#zero_corr = [-28.35+24.75, 16.22-24.75, -39.6+24.75, 26.77-24.75, -33.1+24.75, 14.64-24.75]
#th_abs = [-60, 60, -60, 60, -60, 60]
theta_req1= [90,0,0,0,0,0]

#th_abs = [12.44, -24.48, -8.04, -12.62, 12.35, -14.02]
#th_abs = [-53.49, 22.37, -41.71, 57.98, -56.92, 45.58]

if __name__ == '__main__':
    
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    port = ports[0]
    print(port)
    print("connecting on the first available port:  ports[0]")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    print("dxl_io:"), dxl_io
    ids = dxl_io.scan([1, 2, 3, 4, 5, 6])
    print("Found ids:"), ids
    print("Present motor position"), dxl_io.get_present_position(ids)
    #speed = dict(zip(ids, itertools.repeat(70)))
    dxl_io.set_moving_speed({4: 100})
    dxl_io.set_goal_position({ 4: -80})
    #dxl_io.set_goal_position({1: th_abs[0], 2: th_abs[1], 3: th_abs[2], 4: th_abs[3], 5: th_abs[4], 6: th_abs[5]})
    tt = dxl_io.get_control_table([1])
    print("Control mode\n", tt[0]['present current'])
    

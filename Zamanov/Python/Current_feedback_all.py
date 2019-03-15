import itertools
import numpy
import time
import pypot.dynamixel
import matplotlib.pyplot as plt
import xlwt

zero_corr = [-28.35+24.75, 16.22-24.75, -39.6+24.75, 26.77-24.75, -33.1+24.75, 14.64-24.75]
#th_abs = [-60, 60, -60, 60, -60, 60]
theta_req1= [0,0,0,0,0,0]
#th_abs = [31.78, -20.09, 3.82, 136.92, -0.75, -169.71]
#th_abs = [-56.75, 34.33, -51.03, 41.71, -41.01, 51.91]
time_feed = []
current_list = []
load_list = []
current_feed = []
load_feed = []
        

if __name__ == '__main__':
    
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    port = ports[0]
    print("connecting on the first available port:  ports[0]")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    print("dxl_io:"), dxl_io
    ids = dxl_io.scan([6])
    print("Found ids:"), ids
    print("Present motor position"), dxl_io.get_present_position(ids)
    speed = dict(zip(ids, itertools.repeat(10)))
    #dxl_io.set_moving_speed({6: 10})
    #dxl_io.set_goal_position({1: (theta_req1[0]+zero_corr[0]), 2: (theta_req1[1]+zero_corr[1]), 3: (theta_req1[2]+zero_corr[2]), 4: (theta_req1[3]+zero_corr[3]), 5: (theta_req1[4]+zero_corr[4]), 6: (theta_req1[5]+zero_corr[5])})
    #dxl_io.set_goal_position({6: 0})
    #dxl_io.set_goal_position({1: th_abs[0], 2: th_abs[1], 3: th_abs[2], 4: th_abs[3], 5: th_abs[4], 6: th_abs[5]})
    '''    
    t0 = time.time()
    tr = 0
    while((tr-t0) < 30):
        tr  = time.time()
        current_list = []
        load_list = []
        dt = round((tr-t0),3)
        for i in range(0,len(ids)):
            control_table = dxl_io.get_control_table([ids[i]])
            current_list.append(control_table[0]['present current'])
            load_list.append(control_table[0]['present load'])
        current_feed.append(current_list)
        load_feed.append(load_list)
        time_feed.append(dt)
        print("Current time\n", dt)
        #Plotting
        ax = plt.gca()
        ax.set_ylim([-2, 2])
        ax.set_xlim([0, 30])
        plt.plot(time_feed, current_feed, 'r-')
        plt.draw()
        plt.pause(0.01)

    #Saving data in xls 
    book = xlwt.Workbook(encoding="utf-8")
    sheet1 = book.add_sheet("Time")
    sheet2 = book.add_sheet("Current")
    sheet3 = book.add_sheet("Load")
    print(len(current_feed))
    print(len(current_feed[0]))
    for i in range(0,len(current_feed)):
        sheet1.write(i,0,time_feed[i])
        for j in range(0,len(current_feed[0])):
            sheet2.write(i,j,current_feed[i][j])
            sheet3.write(i,j,load_feed[i][j])
    
    
    book.save("Current_Feedback_Mar7_w_Force2.xls")
    '''
    

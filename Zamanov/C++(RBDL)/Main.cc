/*
 * RUS6 - Parallel Mechanism - Zamanov Type interface with Hardware
 * Author: Anirvan Dutta & Durgesh Salunkhe  <www.anirvandutta.com> & <www.salunkhedurgesh.com>
 * Acknowledgement: Shubham Paul @GreyOrange PVT Ltd, Shivesh Kumar @DFKI Germany.
 */

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/time.h>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif


#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
  #error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>
#include <RUS6.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <dynamixel_sdk.h>

#include <math.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;
using Eigen::MatrixXd;
using namespace RUS6;

#define pi 3.14159265

// ============================================================Dynamixel Control definitions ==================================================
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_VELOCITY         32
#define ADDR_MX_PRESENT_VELOCITY        38
#define ADDR_MX_PRESENT_CURRENT         68
#define ADDR_MX_PRESENT_LOAD            40
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_MOVING_VELOCITY          2

// ===================================================== Secondary Functions ======================================================
int speed_to_dxl(double value)
{
    int direction, speed;
    double max_value, speed_factor;
    if(value < 0)
        direction = 1024;
    else
        direction = 0;
    speed_factor = 0.114;
    max_value = 1023*speed_factor*6;
    value = min(max(value, -max_value), max_value);
    speed = (round(direction + abs(value)/(6*speed_factor)));
    return speed;
}

int degree_to_dxl(double degreeval)
{
  int max_pos, max_deg, pos;
  max_pos = 4096;
  max_deg = 360;
  pos = (round((max_pos - 1)*((max_deg/2 + degreeval)/max_deg)));
  pos = min(max(pos, 0), max_pos - 1);
  return pos;
}

double dxl_to_degree(int dxlval)
{
  int max_pos, max_deg;
  double pos;
  max_pos = 4096;
  max_deg = 360;
  pos = ((max_deg*dxlval)/(max_pos - 1)) - (max_deg/2);
  return pos;
}

void check_error(int dxl_comm_re, uint8_t dxl_error)
{
  if (dxl_comm_re != COMM_SUCCESS)
    printf("Dxl comm failed\n");
  else if (dxl_error != 0)
    printf("Dxl error failed\n");
}

// ============================================================= main function =====================================================

int main (int argc, char* argv[])
{
  rbdl_check_api_version (RBDL_API_VERSION);
  double dt, Tf;
  time_t initial_time, current_time;
	dt = 0.05;
	Tf = 100;
  Model m;
	if (!Addons::LuaModelReadFromFile ("Zamanov_mechanism_dhs.lua", &m, false))
	{
		std::cerr << "Error loading model Zamanov_mechanism_dhs.lua" << std::endl;
		abort();
	}

  ofstream tau_file("tau_actuated_force_removal_1.csv");    //Opening file to print info to write tau actuated
  ofstream feedback_file("feedback_file_force_removal_1.csv");
  ofstream wrench_file("wrench_file_force_removal_1.csv");

  VectorXd Q = VectorXd::Zero(m.dof_count);
  VectorXd theta_one = VectorXd::Zero(6);
  VectorXd Qdot = VectorXd::Zero(m.dof_count);
  VectorXd Udot = VectorXd::Zero(6);
  VectorXd Qddot = VectorXd::Zero(m.dof_count);
  VectorXd Q_prev = VectorXd::Zero(m.dof_count);
  VectorXd Qdot_Num = VectorXd::Zero(m.dof_count);
  VectorXd Qdot_an = VectorXd::Zero(m.dof_count);
  VectorXd error = VectorXd::Zero(m.dof_count);
  VectorXd y = VectorXd::Zero(6);
  VectorXd ydot = VectorXd::Zero(6);
  VectorXd Y_D = VectorXd::Zero(6);
  VectorXd yddot = VectorXd::Zero(6);
  VectorXd wrench = VectorXd::Zero(6);
  VectorXd Tau_spanningtree(m.dof_count);
  VectorXd Tau_actuated(6);
	MatrixXd G_matrix(21,6), Gu(6,6), Gu_inv(6,6);
  int i, j, k;
  RUS6::rus6 mechanism("Zamanov_mechanism_dhs.lua");

  //=======================================================  Dynamixel Motor Functions ========================================
  VectorXd zero_corr(6), dxl_id(6), tor_map(6), tor_diff(6);
  zero_corr << -8.0900, -7.4800, -14.8500, 1.0500, -7.9100, -9.6700;
  dxl_id << 1,2,3,4,5,6;
  tor_map << 0.017, 0.0219, 0.0203, 0.0212, 0.0196, 0.0209;
  VectorXd present_current(6), prev_load(6);
  VectorXd present_load = VectorXd::Zero(6);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  bool dxl_addparam_result = false;
  bool force_detect = false;
  bool intialised_flag = false;
  uint8_t param_moving_velocity[2];
  uint8_t param_goal_position[2];
  uint16_t dxl_present_current, dxl_present_load;

  //variables for impedance_control
  int force_applied, force_removed, first_case_force_applied, first_case_force_removed, system_set;
  double time_force_applied, time_force_removed, t ,t0, slope;
  force_applied = 0;
  force_removed = 0;
  first_case_force_applied = 1;
  first_case_force_removed = 1;
  VectorXd ye = VectorXd::Zero(6);
  VectorXd y0 = VectorXd::Zero(6);
  VectorXd y0dot = VectorXd::Zero(6);
  VectorXd yedot = VectorXd::Zero(6);
  VectorXd yeddot = VectorXd::Zero(6);
  VectorXd force(6);
  system_set = 1;

  t = 0;
  // Initialize PortHandler instance
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  // Initialize PacketHandler instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort())
    printf("Succeeded to open the port!\n");
  else
  {
    printf("Failed to open the port!\n");
    return 0;
  }
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
    printf("Succeeded to change the baudrate!\n");
  else
  {
    printf("Failed to change the baudrate!\n");
    return 0;
  }
  // Enable Dynamixel Torques
  for(i = 0;i<6;i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id(i), ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    check_error(dxl_comm_result, dxl_error);
  }

  // Initialize GroupSyncWrite instance for Velocity
  dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, ADDR_MX_MOVING_VELOCITY, LEN_MX_MOVING_VELOCITY);
  // Provide moving velocities
  // Allocate moving velocity value into byte array
  param_moving_velocity[0] = DXL_LOBYTE(speed_to_dxl(15));
  param_moving_velocity[1] = DXL_HIBYTE(speed_to_dxl(15));
  for(i = 0;i<6;i++)
  {
    dxl_addparam_result = groupSyncWriteVel.addParam(dxl_id(i), param_moving_velocity);
    if (dxl_addparam_result != true)
    {
     fprintf(stderr, "GroupSyncWrite addparam failed\n");
    }
  }
  dxl_comm_result = groupSyncWriteVel.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  // Clear syncwrite parameter storage
  groupSyncWriteVel.clearParam();
  // Initialize GroupSyncWrite instance for Position
  dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
  // ================================================= LOOP ===================================================================
  initial_time = time(0);
  current_time = time(0);
  while ((current_time-initial_time) <= (Tf+dt) && system_set == 1)
	{
    cout << "\n Current time: " << double(current_time-initial_time);
    //========================================== Defining trajectory ==========================================================
    if(force_applied == 0)
    {
    	y(0) = 0.2215;
      y(1) = 0.00*cos(8*t);
    	y(2) = 0.00*sin(8*t);
    	y(3) = 0.0;
    	y(4) = 0.0;
    	y(5) = 0.0;
      ydot(0) = -0.0*sin(20*t);
      ydot(1) = -0.00*sin(8*t);
      ydot(2) = 0.00*cos(8*t);
      ydot(3) = -0.0*sin(t);
      ydot(4) = 0.0*cos(t);
      ydot(5) = 0;
    }
    else if(force_applied == 1)
    {
      if(first_case_force_applied == 1)
      {
        t0 = double(current_time-initial_time);
        y0 = y;
        ye = y;
        force << -(int)(wrench(0) - 10),-(int)(wrench(1))*2,-(int)(wrench(2))*2,0,0,0;
        //force << 0,0,0,0,0,-wrench(5)*2.5;
        first_case_force_applied = 0;
      }
      t = double(current_time - initial_time);
      y = mechanism.rus6::impedance_control(t, t0, force, y0, ye, y0dot, yedot, yeddot,false);
      cout <<"\nTarget :: " << y;
      cout << "\nTime:: " << t;
    }
    else if(force_removed == 1)
    {
      if(first_case_force_removed == 1)
      {
        t0 = double(current_time-initial_time);
        y0 = y;
        ye = y;
        force << 0,0,0,0,0,0;
        first_case_force_removed = 0;
      }
      t = double(current_time - initial_time);
      y = mechanism.rus6::impedance_control(t, t0, force, y0, ye, y0dot, yedot, yeddot,true);
    }

	  //cout << Q << endl;
    Q = mechanism.rus6::calc_sysstate_q(y);
    Qdot = mechanism.rus6::calc_sysstate_qdot(y,ydot);
    Qddot = mechanism.rus6::calc_sysstate_qddot(y,ydot,yddot);
    G_matrix = mechanism.rus6::calc_loopclosure_Jacobian(y);
    Udot(0) = Qdot(0);
    Udot(1) = Qdot(6);
    Udot(2) = Qdot(9);
    Udot(3) = Qdot(12);
    Udot(4) = Qdot(15);
    Udot(5) = Qdot(18);
    theta_one(0) = Q(0)*180/pi+90 + zero_corr(0);
    theta_one(1) = -(Q(6)*180/pi+90) + zero_corr(1);
    theta_one(2) = Q(9)*180/pi+90 + zero_corr(2);
    theta_one(3) = -(Q(12)*180/pi+90) + zero_corr(3);
    theta_one(4) = Q(15)*180/pi+90 + zero_corr(4);
    theta_one(5) = -(Q(18)*180/pi+90) + zero_corr(5);
    Q = mechanism.rus6::calc_sysstate_q(y);
    if(intialised_flag==false)
    {
      cout << "Initialised\n";
      Q_prev = Q;
      intialised_flag = true;
    }
    else
    {
      Qdot_Num = (Q-Q_prev)/dt;
      error = Qdot - Qdot_Num;
      //cout<<"Qdot Analytical"<< Qdot << endl;
      //cout<<"Qdot Numerical"<< Qdot_Num << endl;
      //cout<<"error"<< error << endl;
      Q_prev = Q;
      //extracting G matrix corresponding to actuated joints
      for(j=0;j<=5;j++)
      {
        Gu(0,j) = G_matrix(0,j);
      }
      k = 1;
      for(i=6;i<=18;i=i+3)
      {
        for(j=0;j<=5;j++)
        {
          Gu(k,j) = G_matrix(i,j);
        }
        k++;
      }
      //cout << "Gu determinant: "<<Gu.determinant()<<endl;
      InverseDynamics(m, Q, Qdot, Qddot, Tau_spanningtree);
      Gu_inv = Gu.inverse();
      Y_D = Gu_inv*Udot;
      //cout<<"error"<<Y_D-ydot<<endl;
      Tau_actuated = Gu_inv.transpose() * G_matrix.transpose() * Tau_spanningtree;

      //cout << "Wrench : " << wrench << endl;
      //cout<<"Tau_actuated\n"<< Tau_actuated[0] << ":" << Tau_actuated[1] << ":" << Tau_actuated[2] << ":" << Tau_actuated[3] << ":" << Tau_actuated[4] << ":" <<Tau_actuated[5] << endl;
      //tau_file << t << ":" << Tau_actuated[0] << ":" << Tau_actuated[1] << ":" << Tau_actuated[2] << ":" << Tau_actuated[3] << ":" << Tau_actuated[4] << ":" << Tau_actuated[5] <<endl;
      //============================================= Interface with Hardware =========================================
      // Provide goal position
      for(i = 0;i<6;i++)
      {
        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(degree_to_dxl(theta_one(i)));
        param_goal_position[1] = DXL_HIBYTE(degree_to_dxl(theta_one(i)));
        dxl_addparam_result = groupSyncWritePos.addParam(dxl_id(i), param_goal_position);
        if (dxl_addparam_result != true)
        {
         fprintf(stderr, "GroupSyncWrite addparam failed\n");
        }
      }
      dxl_comm_result = groupSyncWritePos.txPacket();
      if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      // Clear syncwrite parameter storage
      groupSyncWritePos.clearParam();

      prev_load = present_load;
      // Taking Feedback of Current and Load
      for(i = 0;i<6;i++)
      {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id(i), ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);
        check_error(dxl_comm_result, dxl_error);
        if(dxl_present_load > 1024)
          present_load(i) = -pow(-1,dxl_id(i))*(dxl_present_load - 1024)*0.1*5*tor_map(i);
        else
          present_load(i) = pow(-1,dxl_id(i))*dxl_present_load*0.1*5*tor_map(i);
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id(i), ADDR_MX_PRESENT_CURRENT, &dxl_present_current, &dxl_error);
        present_current(i) = 4.5*(dxl_present_current - 2048)/1000;
      }
      if((current_time-initial_time)>2 && force_applied == 0 && force_removed ==0)
      {
        cout << "\nSystem set\n";
        tor_diff = present_load - Tau_actuated;
        //cout << "\nActual_load : " << present_load;
        //cout << "\nTheo_load : " << Tau_actuated;
        wrench = Gu.transpose()*tor_diff;
        //cout << "\n Wrench \n" << wrench;
        if(abs(tor_diff(0)) > 0.25 || abs(tor_diff(1)) > 0.25 || abs(tor_diff(2)) > 0.25 || abs(tor_diff(3)) > 0.25 || abs(tor_diff(4)) > 0.25 || abs(tor_diff(5)) > 0.25)
        {
          usleep(200000);
          if(abs(tor_diff(0)) > 0.25 || abs(tor_diff(1)) > 0.25 || abs(tor_diff(2)) > 0.25 || abs(tor_diff(3)) > 0.25 || abs(tor_diff(4)) > 0.25 || abs(tor_diff(5)) > 0.25)
          {
            cout << " Force dectected\n";
            //usleep(2000000);
            force_applied = 1;
            time_force_applied = current_time - initial_time;
          }
          else
          {
            cout << "Noise detected\n";
          }
        }
      }
      else if(force_applied == 1 || force_removed == 1)
      {
        slope = 0;
        for(i = 0;i<6;i++)
        {
          slope = slope + abs(present_load(i)-prev_load(i));
        }
        slope = slope/6;
        cout << "\n Slope: " << slope <<endl;
        if((((current_time-initial_time) - time_force_applied)> 3) && force_applied == 1)
        {
          cout << "Force Removed\n";
          force_applied = 0;
          force_removed = 1;
          time_force_removed = current_time - initial_time;
        }
        else if((((current_time-initial_time) - time_force_removed) > 3) && force_removed == 1)
        {
          cout << "System reset\n";
          force_removed = 0;
          system_set = 0;
          first_case_force_removed = 0;
          first_case_force_applied = 0;
        }
      }
      feedback_file << (current_time-initial_time) << ", " << present_load(0) << "," << present_load(1) << "," << present_load(2) << "," << present_load(3) << "," << present_load(4) << "," << present_load(5) << "," << "," << present_current(0) << "," << present_current(1) << "," << present_current(2) << "," << present_current(3) << "," << present_current(4) << "," << present_current(5) << endl;
      wrench_file << (current_time-initial_time) << ":" << wrench(0) << ":" << wrench(1) << ":" << wrench(2) << ":" << wrench(3) << ":" << wrench(4) << ":" << wrench(5) << endl;
      tau_file << (current_time-initial_time) << ":" << Tau_actuated(0) << ":" << Tau_actuated(1) << ":" << Tau_actuated(2) << ":" << Tau_actuated(3) << ":" << Tau_actuated(4) << ":" << Tau_actuated(5) << endl;

    }
	   //incrementing time
     current_time = time(0);
     //t = t+dt;
   }

   // Close port
   portHandler->closePort();
   return 0;
}

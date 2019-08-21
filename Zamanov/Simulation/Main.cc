/*
 * RUS6 - Parallel Mechanism - Zamanov Type Simulation
 * Author: Anirvan Dutta & Durgesh Salunkhe  <www.anirvandutta.com> & <www.salunkhedurgesh.com>
 * Acknowledgement: Shubham Paul @GreyOrange PVT Ltd, Shivesh Kumar @DFKI Germany.
 */

#include <iostream>
#include <fstream>

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
  #error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>
#include <RUS6.hpp>

#include <math.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;
using Eigen::MatrixXd;
using namespace RUS6;

#define pi 3.14159265


int main (int argc, char* argv[])
{
  rbdl_check_api_version (RBDL_API_VERSION);
  double t, dt, Tf;
  t = 0.0;
  dt = 0.05;
  Tf = 10;
  Model m;

  if (!Addons::LuaModelReadFromFile ("Zamanov_mechanism_dhs.lua", &m, false))
	{
		std::cerr << "Error loading model Zamanov_mechanism_dhs.lua" << std::endl;
		abort();
	}



  cout<<"Model DoF overview:"<<Utils::GetModelDOFOverview(m)<<endl;
  cout<<"Model Hierarchy overview:"<<Utils::GetModelHierarchy(m)<<endl;
  cout<<"Named Body Origins overview:"<<Utils::GetNamedBodyOriginsOverview(m)<<endl;

  cout << "Gravity: " << endl << m.gravity << endl;

  cout << "DoF count: " << m.dof_count << endl;
  // Set all animation files
  ofstream animation_file("zamanov_mechanism_animation.csv");    //Opening file to print info to
  ofstream tau_file("tau_actuated.csv");    //Opening file to print info to

  animation_file << "COLUMNS:" << endl;          //Headings for file
  animation_file << "Time, " << endl;          //Headings for file
  animation_file << "Rjoint_cyl1:r:z:rad, " << endl;
  animation_file << "Ujoint_z1:r:z:rad, " << endl;
  animation_file << "Ujoint_y1:r:y:rad, " << endl;
  animation_file << "Sjoint_z1:r:z:rad, " << endl;
  animation_file << "Sjoint_y1:r:y:rad, " << endl;
  animation_file << "Sjoint_x1:r:x:rad, " << endl;
  animation_file << "Rjoint_cyl2:r:z:rad, " << endl;
  animation_file << "Ujoint_z2:r:z:rad, " << endl;
  animation_file << "Ujoint_y2:r:y:rad, " << endl;
  animation_file << "Rjoint_cyl3:r:z:rad, " << endl;
  animation_file << "Ujoint_z3:r:z:rad, " << endl;
  animation_file << "Ujoint_y3:r:y:rad, " << endl;
  animation_file << "Rjoint_cyl4:r:z:rad, " << endl;
  animation_file << "Ujoint_z4:r:z:rad, " << endl;
  animation_file << "Ujoint_y4:r:y:rad, " << endl;
  animation_file << "Rjoint_cyl5:r:z:rad, " << endl;
  animation_file << "Ujoint_z5:r:z:rad, " << endl;
  animation_file << "Ujoint_y5:r:y:rad, " << endl;
  animation_file << "Rjoint_cyl6:r:z:rad, " << endl;
  animation_file << "Ujoint_z6:r:z:rad, " << endl;
  animation_file << "Ujoint_y6:r:y:rad, " << endl;

  animation_file << "DATA:" << endl;          //Headings for file

  VectorXd Q = VectorNd::Zero(m.dof_count);
  VectorXd Qdot = VectorNd::Zero(m.dof_count);
  VectorXd Udot = VectorNd::Zero(6);
  VectorXd Qddot = VectorNd::Zero(m.dof_count);
  VectorXd Q_prev = VectorNd::Zero(m.dof_count);
  VectorXd Qdot_Num = VectorNd::Zero(m.dof_count);
  VectorXd Qdot_an = VectorNd::Zero(m.dof_count);
  VectorXd error = VectorNd::Zero(m.dof_count);
  VectorXd y = VectorNd::Zero(6);
  VectorXd ydot = VectorNd::Zero(6);
  VectorXd Y_D = VectorNd::Zero(6);
  VectorXd yddot = VectorNd::Zero(6);
  VectorXd Tau_spanningtree(m.dof_count);
  VectorXd Tau_actuated(6);
	RUS6::rus6 mechanism("Zamanov_mechanism_dhs.lua");
  MatrixXd G_matrix(21,6), Gu(6,6), Gu_inv(6,6);
  int i, j, k;

  while (t <= (Tf+dt))
  	{
  		y(0) = 0.18;
  		y(1) = 0.04*cos(t);
  		y(2) = 0.04*sin(t);
  		y(3) = 0.0*cos(t);
  		y(4) = 0.0*sin(t);
  		y(5) = 0;
      ydot(0) = 0;
      ydot(1) = -0.04*sin(t);
      ydot(2) = 0.04*cos(t);
      ydot(3) = -0.0*sin(t);
      ydot(4) = 0.0*cos(t);
      ydot(5) = 0;
      yddot(0) = 0;
      yddot(1) = -0.04*cos(t);
      yddot(2) = -0.04*sin(t);
      yddot(3) = -0.0*cos(t);
      yddot(4) = -0.0*sin(t);
      yddot(5) = 0;
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


      if(t==0)
      {
        cout << "Initialised\n";
        Q_prev = Q;
      }
      else
      {

        Qdot_Num = (Q-Q_prev)/dt;
        error = Qdot - Qdot_Num;
        //cout<<"Qdot Analytical"<< Qdot_an << endl;
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
      //cout << "Q"<< Q << endl;
      InverseDynamics(m, Q, Qdot, Qddot, Tau_spanningtree);
      Gu_inv = Gu.inverse();
      //cout << Udot.size();
      Y_D = Gu_inv*Udot;
      //cout<<"error"<<Y_D-ydot<<endl;
      Tau_actuated = Gu_inv.transpose() * G_matrix.transpose() * Tau_spanningtree;
      //cout<<"Tau_spanningtree"<< Tau_spanningtree << endl;
      //cout<<"Tau_actuated"<< Tau_actuated << endl;
  		animation_file << t << ", " << Q[0] << ", " << Q[1] << ", " << Q[2] << ", " << Q[3] << ", " << Q[4] << ", " << Q[5] << ", " << Q[6] << ", " <<  Q[7] << ", " <<  Q[8] << ", " <<  Q[9] << ", " << Q[10] << ", " << Q[11]<< ", "  << Q[12] << ", " << Q[13] << ", " << Q[14] << ", " << Q[15] << ", " << Q[16] << ", " << Q[17] << ", " << Q[18] << ", " << Q[19] << ", " << Q[20] << endl;
      //incrementing time
      tau_file << t << ":" << Tau_actuated[0] << ":" << Tau_actuated[1] << ":" << Tau_actuated[2] << ":" << Tau_actuated[3] << ":" << Tau_actuated[4] << ":" << Tau_actuated[5] <<endl;
    }
    t = t + dt;
  	}

    return 0;
  }

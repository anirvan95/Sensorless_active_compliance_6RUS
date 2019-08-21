#ifndef RUS6_H
#define RUS6_H

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>

#include <fstream>

#include <rbdl/rbdl.h>

/*#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>*/

#ifndef RBDL_BUILD_ADDON_LUAMODEL
  #error "Error: RBDL addon LuaModel not enabled."
#endif

#include <rbdl/addons/luamodel/luamodel.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
//using Eigen::Vector3d;
using Eigen::VectorXd;

namespace RUS6
{
	class rus6
	{
		public:
		  double l1, l2, rem_angle, half_angle, base_length, top_length, theta_b, theta_p, r_b, r_p, rem_angle_top;     // 6 RUS hexa stewart mechanism parameters
      unsigned int dof_active, dof_spanningtree;

      // Constructor
      rus6(string path_lua);
      double wrap2pi(double x);
      // Manually coded
			VectorXd calc_loopclosure_function(const Math::VectorNd y);
			MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd y);
      MatrixXd calc_loopclosure_Jacobiandot(const Math::VectorNd y, const Math::VectorNd ydot);
			//MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd y);
			VectorXd calc_sysstate_q(const Math::VectorNd y);
			MatrixXd calc_sysstate_qdot(const Math::VectorNd y, const Math::VectorNd ydot);
			MatrixXd calc_sysstate_qddot(const Math::VectorNd y, const Math::VectorNd ydot, const Math::VectorNd yddot);
			//Declaring a function for rotating a frame about z-axis in clockwise direction
			MatrixXd rotation_x(double angle);
  };

} // end namespace RUS6

#endif // rus6

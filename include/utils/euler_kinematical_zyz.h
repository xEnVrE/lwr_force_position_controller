// Author: Nicola Piga, Giulio Romualdi
//
// eul_kin_ZYZ() computes the matrix T(PHI) such that omega = T PHI_dot
// where omega is the angular velocity vector  of the body whose attitude is represented by the angles PHI (ZYZ)
// omega is projected with the rotation matrix R(PHI)
// returns the matrix T
//
// eul_kin_ZYZ_dot() computes the matrix d/dt{T(PHI)}


#ifndef EULER_KIN_RPY_H
#define EULER_KIN_RPY_H

#include <math.h>
#include <Eigen/LU>
using namespace Eigen;

inline void eul_kin_ZYZ(const double beta, const double alpha, Eigen::Matrix3d &T)
{	
  T << 
    0, -sin(alpha), cos(alpha) * sin(beta),
    0, cos(alpha), sin(alpha) * sin(beta),
    1, 0, cos(beta);
}

inline void eul_kin_ZYZ_dot(const double beta, const double alpha,
			    const double beta_dot, const double alpha_dot,
			    Eigen::Matrix3d &T_dot)
{	
    T_dot <<
      0, -cos(alpha) * alpha_dot, -sin(alpha) * sin(beta) * alpha_dot + cos(alpha) * cos(beta) * beta_dot,
      0, -sin(alpha) * alpha_dot, cos(alpha) * sin(beta) * alpha_dot + cos(beta) * sin(alpha) * beta_dot,
      0, 0, -sin(beta) * beta_dot;
}

#endif

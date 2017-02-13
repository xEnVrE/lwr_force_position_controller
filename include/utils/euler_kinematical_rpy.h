// Author: Nicola Piga, Giulio Romualdi
//
// eul_kin_RPY() computes the matrix T(PHI) such that omega = T PHI_dot
// where omega is the angular velocity vector  of the body whose attitude is represented by the angles PHI (RPY)
// omega is projected with the rotation matrix R(PHI)
// returns the matrix T
//
// eul_kin_RPY_dot() computes the matrix d/dt{T(PHI)}


#ifndef EULER_KIN_RPY_H
#define EULER_KIN_RPY_H

#include <math.h>
#include <Eigen/LU>
using namespace Eigen;

inline void eul_kin_RPY(const double pitch, const double yaw, Eigen::Matrix3d &T)
{	
  T << 
    0, -sin(yaw), cos(pitch) * cos(yaw),
    0, cos(yaw), cos(pitch) * sin(yaw),
    1, 0, -sin(pitch);
}

inline void eul_kin_RPY_dot(const double pitch, const double yaw,
			    const double pitch_dot, const double yaw_dot,
			    Eigen::Matrix3d &T_dot)
{	
    T_dot <<
	0, -cos(yaw) * yaw_dot, -cos(yaw) * sin(pitch) * pitch_dot - cos(pitch) * sin(yaw) * yaw_dot,
	0, -sin(yaw) * yaw_dot, -sin(pitch) * sin(yaw) * pitch_dot + cos(pitch) * cos(yaw) * yaw_dot,
	0, 0, -cos(pitch) * pitch_dot;
}


#endif

// Author: Nicola Piga, Giulio Romualdi
//
// skew_pinv evaluate() the pseudo inverse of a skew symmetric matrix
// vector is the vector whose skew symmetric is of interest
// returns pinv(skew(vector)) in the matrix pinv

#ifndef SKEW_PINV
#define SKEW_PINV

#include <Eigen/LU>
using namespace Eigen;

inline void skew_pinv(Eigen::Vector3d &vector, Eigen::Matrix3d &pinv)
{
  double denominator = vector.squaredNorm();
  double a = vector(0);
  double b = vector(1);
  double c = vector(2);
  
  pinv <<
    0, c, -b,
    -c, 0, a,
    b, -a, 0;
  
  pinv /= denominator;
}

#endif

#ifndef __PCL_ALIGN_HPP__
#define __PCL_ALIGN_HPP___

#include <Eigen/Geometry>
#include <vector>

namespace cvx { namespace util {

// Kabsch algorithm for rigid pose estimation between two point clouds ( finds T to minimize sum_i ||P_i * T - Q_i|| )

Eigen::Isometry3f alignRigid(const Eigen::Matrix3Xf &P, const Eigen::Matrix3Xf &Q) ;
Eigen::Isometry3f alignRigid(const std::vector<Eigen::Vector3f> &P, const std::vector<Eigen::Vector3f> &Q) ;

}}

#endif

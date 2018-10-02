#ifndef __INTERSECT_HPP__
#define __INTERSECT_HPP__

#include <Eigen/Core>
#include <cvx/viz/scene/camera.hpp>

namespace cvx { namespace viz { namespace detail {


class AABB
{
    public:
    AABB(const Eigen::Vector3f &b0, const Eigen::Vector3f &b1) {
        bounds_[0] = b0 ;
        bounds_[1] = b1;
    }

    Eigen::Vector3f bounds_[2];
};

bool rayIntersectsAABB(const Ray &, const AABB &box, float &t) ;
bool rayIntersectsTriangle(const Ray ray,
                           const Eigen::Vector3f &v0,
                           const Eigen::Vector3f &v1,
                           const Eigen::Vector3f &v2,
                           bool back_face_culling,
                           float &t) ;
bool rayIntersectsSphere(const Ray &ray, const Eigen::Vector3f &center, float radius, float &t) ;

bool triangleInsideBox(const Eigen::Vector3f &tv0, const Eigen::Vector3f &tv1,
                   const Eigen::Vector3f &tv2, const Eigen::Vector3f &boxcenter, const Eigen::Vector3f &boxhalfsize) ;

bool triangleIntersectsBox(const Eigen::Vector3f &tv0, const Eigen::Vector3f &tv1,
                   const Eigen::Vector3f &tv2, const Eigen::Vector3f &boxcenter, const Eigen::Vector3f &boxhalfsize) ;
}}}
#endif

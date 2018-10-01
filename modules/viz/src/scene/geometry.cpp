#include <cvx/viz/scene/geometry.hpp>
#include <iostream>

#include "intersect.hpp"

using namespace std ;

namespace cvx { namespace viz {

using namespace detail ;

bool BoxGeometry::intersect(const cvx::viz::Ray &ray, float &t) const {
    AABB box(-half_extents_, half_extents_) ;
    return rayIntersectsAABB(ray, box, t) ;
}

bool SphereGeometry::intersect(const cvx::viz::Ray &ray, float &t) const {
     return rayIntersectsSphere(ray, {0, 0, 0}, radius_, t) ;
}


}}

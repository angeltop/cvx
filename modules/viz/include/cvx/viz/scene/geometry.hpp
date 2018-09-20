#ifndef __CVX_VIZ_GEOMETRY_HPP__
#define __CVX_VIZ_GEOMETRY_HPP__

#include <cvx/viz/scene/scene_fwd.hpp>

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>

namespace cvx {

// abstract geometry class

class Geometry {
public:
    Geometry() = default ;
    virtual ~Geometry() {}
};


class BoxGeometry: public Geometry {
public:
    BoxGeometry(const Eigen::Vector3f &he): half_extents_(he) {}
    BoxGeometry(float hx, float hy, float hz): half_extents_{hx, hy, hz} {}
    ~BoxGeometry() {}

    Eigen::Vector3f halfExtents() const { return half_extents_ ; }
private:

    Eigen::Vector3f half_extents_ ;
};

class SphereGeometry: public Geometry {
public:
    SphereGeometry(float radius): radius_(radius) {}

    float radius() const { return radius_ ; }

private:
    float radius_ ;
};

class CylinderGeometry: public Geometry {
public:

    CylinderGeometry(float r, float h): radius_(r), height_(h) {}

    float radius() const { return radius_ ; }
    float height() const { return height_ ; }

private:
    float radius_, height_ ;
};

class ConeGeometry: public Geometry {
public:
    ConeGeometry(float r, float h): radius_(r), height_(h) {}

    float radius() const { return radius_ ; }
    float height() const { return height_ ; }

private:
    float radius_, height_ ;
};


} // namespace cvx
#endif

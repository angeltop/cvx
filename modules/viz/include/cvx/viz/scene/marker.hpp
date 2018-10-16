#ifndef __CVX_VIZ_MARKER_HPP__
#define __CVX_VIZ_MARKER_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>

#include <cvx/viz/scene/scene_fwd.hpp>

namespace cvx { namespace viz {

/*
 *  Markers are shapes that are drawn multiple times and therefore share the same geometry
 *  The Marker class implements the common part of marker rendering i.e. vertex buffer filling with geometry
 *  and shader program initialization. They cannot be created directly. Instead the user creates a MarkerInstance which consists of a position and
 *  set of parameters. Parameters are specific to each marker e.g. radius of a sphere, and can be also shared across instances
 *  (e.g. if you want to draw a sphere of constant radius multiple times). These are passed to the associated shader program.
 */

class Marker {
public:

    Marker() = default;
    virtual ~Marker() = default ;
protected:
    virtual void initBuffers() = 0 ;
};

class MarkerParameters {
public:
    virtual ~MarkerParameters() = default ;
    virtual void apply() = 0 ;
};

class MarkerInstance {
public:
    void setCenter(const Eigen::Vector3f &c) { center_ = c ; }

protected:
    MarkerInstance(const std::shared_ptr<Marker> &marker, const std::shared_ptr<MarkerParameters> &params, const Eigen::Vector3f &c):
        marker_(marker), params_(params), center_(c) {}

    std::shared_ptr<Marker> marker_ ;
    std::shared_ptr<MarkerParameters> params_ ;
    Eigen::Vector3f center_ ;
};

using MarkerInstancePtr = std::shared_ptr<MarkerInstance> ;

class SphereMarkerParameters: public MarkerParameters {
public:
    SphereMarkerParameters(float radius, const Eigen::Vector3f &clr = {1, 1, 1}): radius_(radius), color_(clr) {}

    virtual ~SphereMarkerParameters()  = default ;

    void setRadius(float radius) { radius_ = radius ; }
    void setColor(const Eigen::Vector3f &clr) { color_ = clr ; }
private:

    void apply() override ;

    Eigen::Vector3f color_ ;
    float radius_ ;
};

class SphereMarker ;

class SphereMarkerInstance: public MarkerInstance {

public:
    SphereMarkerInstance(const Eigen::Vector3f &center, const std::shared_ptr<SphereMarkerParameters> &params) ;
    SphereMarkerInstance(const Eigen::Vector3f &center, float radius, const Eigen::Vector3f &color) ;
};


}}
#endif

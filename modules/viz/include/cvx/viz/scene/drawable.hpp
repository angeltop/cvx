#ifndef __CVX_VIZ_DRAWABLE_HPP__
#define __CVX_VIZ_DRAWABLE_HPP__

#include <cvx/viz/scene/scene_fwd.hpp>

namespace cvx { namespace viz {

// a drawable is a combination of geometry and material

class Drawable{
public:

    Drawable(const GeometryPtr &geom, const MaterialInstancePtr &material):
        geometry_(geom), material_(material) {}

    GeometryPtr geometry() const { return geometry_ ; }
    MaterialInstancePtr material() const { return material_ ; }

    void setMaterial(MaterialInstancePtr mat) { material_ = mat ; }

private:

    GeometryPtr geometry_ ;
    MaterialInstancePtr material_ ;
};

} // namespace viz
} // namespace cvx
#endif

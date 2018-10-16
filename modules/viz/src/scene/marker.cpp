#include <cvx/viz/scene/marker.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

class SphereMarker: public Marker {
public:
    SphereMarker() = default ;
    virtual ~SphereMarker() {}

    static std::shared_ptr<Marker> instance() {
        static std::shared_ptr<Marker> instance_(new SphereMarker()) ;
        return instance_ ;
    }
protected:
    void initBuffers() override {
    }


};

SphereMarkerInstance::SphereMarkerInstance(const Vector3f &center, const std::shared_ptr<SphereMarkerParameters> &params):
MarkerInstance(SphereMarker::instance(),params, center ){

}

SphereMarkerInstance::SphereMarkerInstance(const Eigen::Vector3f &center, float radius, const Eigen::Vector3f &color):
    MarkerInstance(SphereMarker::instance(), std::shared_ptr<MarkerParameters>(new SphereMarkerParameters(radius, color)), center )
{

}

void SphereMarkerParameters::apply()
{

}




}}

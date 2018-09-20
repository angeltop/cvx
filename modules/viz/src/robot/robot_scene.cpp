#include <cvx/viz/robot/robot_scene.hpp>
#include "urdf_loader.hpp"

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

RobotScenePtr RobotScene::loadURDF(const string &filename, const map<string, string> &packages, bool load_collision_geometry) {
    URDFLoader loader(packages, load_collision_geometry) ;
    loader.parse(filename) ;
    return loader.exportScene() ;
}

float RevoluteJoint::setPosition(float pos)
{
    Isometry3f tr ;
    tr.setIdentity() ;
    tr.rotate(AngleAxisf(pos, axis_)) ;
    node_->matrix() = tr ;

    pos = std::max(pos, lower_limit_) ;
    pos = std::min(pos, upper_limit_) ;

    for( uint i=0 ; i<dependent_.size() ; i++ ) {
        float vpos = multipliers_[i] * pos + offsets_[i] ;
        if ( auto j = std::dynamic_pointer_cast<RevoluteJoint>(dependent_[i]) ) {
            j->setPosition(vpos) ;
        }
    }

    return pos ;
}

}}

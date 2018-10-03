#ifndef __CVX_VIZ_SCENE_HPP__
#define __CVX_VIZ_SCENE_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>

#include <Eigen/Core>

#include <cvx/viz/scene/scene_fwd.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/drawable.hpp>
#include <assimp/scene.h>


namespace cvx { namespace viz {

namespace impl {
    class AssimpImporter ;
}



// class defining a complete scene

class Scene: public Node {
public:

    Scene() ;

    void load(const std::string &fname, const NodePtr &parent = nullptr, bool make_pickable = false) ;
    void load(const aiScene *sc, const std::string &fname, const NodePtr &parent = nullptr, bool make_pickable = false) ;

    Eigen::Vector3f geomCenter() const ;
    float geomRadius(const Eigen::Vector3f &center) const ;

};

class SceneLoaderException: public std::runtime_error {

public:

    SceneLoaderException(const std::string &message, const std::string &fname):
        std::runtime_error(message + "(" + fname  + ")") {}
};

}}
#endif

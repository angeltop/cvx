#ifndef __CVX_VIZ_ROBOT_SCENE_HPP__
#define __CVX_VIZ_ROBOT_SCENE_HPP__

#include <cvx/viz/scene/scene.hpp>

class URDFLoader ;

namespace cvx { namespace viz {

class JointNode ;
typedef std::shared_ptr<JointNode> JointNodePtr ;
// encapsulates degrees of freedom information

class JointNode {
public:

    enum Type { Revolute, Prismatic, Floating } ;

    JointNode(Type t): type_(t) {}

    virtual ~JointNode() {}

    Type type() const { return type_ ; }

protected:

    friend class URDFLoader ;

    Type type_ ;
    NodePtr node_ ;
    std::vector<JointNodePtr> dependent_ ;
    std::vector<float> offsets_, multipliers_ ; // offsets and multipliers of dependent nodes
};

class RevoluteJoint: public JointNode {
public:
    RevoluteJoint(): JointNode(Revolute) {}

    float setPosition(float pos) ;

protected:
    friend class URDFLoader ;
    float upper_limit_, lower_limit_ ;
    Eigen::Vector3f axis_ ;
};



class RobotScene ;
typedef std::shared_ptr<RobotScene> RobotScenePtr ;

// a scene with associated joint information
// Joints are represented by inserting nodes between links

class RobotScene: public Scene {
public:

    static RobotScenePtr loadURDF(const std::string &filename, const std::map<std::string, std::string> &packages, bool load_collision_geometry = false) ;

    JointNodePtr getJoint(const std::string &name) {
        auto it = joints_.find(name) ;
        if ( it != joints_.end() ) return it->second ;
        else return nullptr ;
    }

private:
    friend class URDFLoader ;
    std::map<std::string, JointNodePtr> joints_ ;
};

class URDFLoadException: public std::runtime_error {
public:
    URDFLoadException(const std::string &msg): std::runtime_error(msg) {}
};

} // namespace viz
} // namespace cvx

#endif

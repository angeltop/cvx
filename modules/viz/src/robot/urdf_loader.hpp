#ifndef __URDF_LOADER_HPP__
#define __URDF_LOADER_HPP__

#include <map>
#include <pugixml/pugixml.hpp>
#include <cvx/viz/robot/robot_scene.hpp>
#include <cvx/viz/scene/material.hpp>

namespace cvx { namespace viz {


struct Joint {
    std::string parent_, child_, type_, mimic_joint_, name_ ;
    NodePtr node_ ;
    Eigen::Vector3f axis_ ;
    float upper_, lower_, mimic_offset_, mimic_multiplier_ ;
};

class URDFLoader {
public:
    URDFLoader(const std::map<std::string, std::string> package_map, bool load_collision_geometry): package_map_(package_map),
        load_collision_geometry_(load_collision_geometry) {}

    void parse(const std::string &urdf_file) ;
    void parseRobot(const pugi::xml_node &node) ;
    void parseLink(const pugi::xml_node &node) ;
    void parseJoint(const pugi::xml_node &node) ;
    bool buildTree();

    RobotScenePtr exportScene() ;

    Eigen::Isometry3f parseOrigin(const pugi::xml_node &node) ;
    NodePtr parseGeometry(const pugi::xml_node &node, const MaterialInstancePtr &mat, Eigen::Vector3f &sc) ;
    void parseMaterial(const pugi::xml_node &node) ;
    bool resolveUri(const std::string &uri, std::string &path);

    std::string robot_name_ ;
    std::map<std::string, Joint> joints_ ;
    std::map<std::string, NodePtr> links_ ;
    std::map<std::string, std::string> package_map_ ;
    std::vector<MeshPtr> meshes_ ;
    std::map<std::string, MaterialInstancePtr> materials_ ;
    NodePtr root_node_ ;
    bool load_collision_geometry_ ;
};


}}
#endif

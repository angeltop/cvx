#include "urdf_loader.hpp"
#include <cvx/viz/scene/drawable.hpp>
#include <cvx/viz/scene/geometry.hpp>
#include <cvx/viz/scene/material.hpp>
#include <cvx/util/misc/strings.hpp>
#include <cvx/util/misc/path.hpp>
#include <set>
using namespace std ;
using namespace pugi ;

using namespace cvx ;
using namespace Eigen ;

namespace cvx { namespace viz {


void URDFLoader::parse(const string &urdf_file) {
    xml_document doc ;

    xml_parse_result result = doc.load_file(urdf_file.c_str()) ;

    if ( !result )
        throw URDFLoadException(result.description()) ;

    xml_node root = doc.child("robot") ;

    if ( !root )
        throw URDFLoadException("No <robot> element found") ;

    robot_name_ = root.attribute("name").as_string() ;

    parseRobot(root) ;


}

void URDFLoader::parseRobot(const xml_node &node) {

    for( const xml_node &n: node.children("material") )
        parseMaterial(n) ;

    for( const xml_node &n: node.children("link") )
        parseLink(n) ;

    for( const xml_node &n: node.children("joint") )
        parseJoint(n) ;
}

void URDFLoader::parseLink(const xml_node &node) {

    NodePtr link(new Node) ;

    string name = node.attribute("name").as_string() ;

    if ( name.empty() )
        throw URDFLoadException("Attribute \"name\" missing from <link>") ;

    link->setName(name) ;


    if ( !load_collision_geometry_ ) {

        if ( xml_node visual_node = node.child("visual") ) {
            Isometry3f tr ;
            tr.setIdentity() ;

            if ( xml_node origin_node = visual_node.child("origin") )
                tr = parseOrigin(origin_node) ;

            NodePtr visual(new Node), geom ;
            visual->setName("visual") ;

            MaterialPtr mat ;

            if ( xml_node material_node = visual_node.child("material") ) {
                string matid = material_node.attribute("name").as_string() ;
                if ( !matid.empty() ) {
                    auto it = materials_.find(matid) ;
                    if ( it != materials_.end() )
                        mat = it->second ;
                }
            }

            Vector3f scale{1, 1, 1} ;

            if ( xml_node geom_node = visual_node.child("geometry") )
                geom = parseGeometry(geom_node, mat, scale) ;
            else
                throw URDFLoadException("<geometry> element is missing from <visual>") ;

            tr.linear() *= scale.asDiagonal() ;
            geom->matrix() = tr ;


            visual->addChild(geom) ;
            link->addChild(visual) ;
        }
    } else {

        if ( xml_node collision_node = node.child("collision") ) {
            Isometry3f tr ;
            tr.setIdentity() ;

            if ( xml_node origin_node = collision_node.child("origin") )
                tr = parseOrigin(origin_node) ;

            NodePtr collision(new Node), geom ;
            collision->setName("collision") ;

            Vector3f scale{1, 1, 1} ;

            if ( xml_node geom_node = collision_node.child("geometry") )
                geom = parseGeometry(geom_node, MaterialPtr(), scale) ;
            else
                throw URDFLoadException("<geometry> element is missing from <collision>") ;

            tr.linear() *= scale.asDiagonal() ;
            geom->matrix() = tr ;


            collision->addChild(geom) ;
            link->addChild(collision) ;
        }




    }

    links_.insert({name, link}) ;
}

static Vector3f parse_vec3(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z ;
    strm >> x >> y >> z ;
    return {x, y, z} ;
}

static Vector4f parse_vec4(const std::string &s) {
    istringstream strm(s) ;
    float x, y, z, w ;
    strm >> x >> y >> z >> w;
    return {x, y, z, w} ;
}


void URDFLoader::parseJoint(const xml_node &node) {
    string name = node.attribute("name").as_string() ;
    string type = node.attribute("type").as_string() ;

    if ( name.empty() )
        throw URDFLoadException("<joint> is missing \"name\" attribute") ;

    Joint j ;

    j.type_ = type ;
    j.name_ = name ;

    NodePtr joint(new Node) ;
    joint->setName(name) ;

    if ( xml_node origin_node = node.child("origin") )
        joint->matrix() = parseOrigin(origin_node) ;

    if ( xml_node parent_node = node.child("parent") ) {
        string link_name = parent_node.attribute("link").as_string() ;
        j.parent_ = link_name ;
    }

    if ( xml_node child_node = node.child("child") ) {
        string link_name = child_node.attribute("link").as_string() ;
        j.child_ = link_name ;
    }

    j.node_ = joint ;

    if ( xml_node axis_node = node.child("axis") ) {
        string axis_str = axis_node.attribute("xyz").as_string() ;
        j.axis_ = parse_vec3(axis_str) ;
    }

    if ( xml_node limits_node = node.child("limit") ) {
        j.lower_ = limits_node.attribute("lower").as_float(0) ;
        j.upper_ = limits_node.attribute("upper").as_float(0) ;

    }

    if ( xml_node mimic_node = node.child("mimic") ) {
        j.mimic_joint_ = mimic_node.attribute("joint").as_string() ;
        j.mimic_offset_ = mimic_node.attribute("offset").as_float(0.0) ;
        j.mimic_multiplier_ = mimic_node.attribute("multiplier").as_float(1.0) ;
    }

    joints_.emplace(name, j) ;

}

bool URDFLoader::buildTree() {
    map<string, string> parent_link_tree ;

    for( auto &jp: joints_ ) {
        string parent_link_name = jp.second.parent_ ;
        string child_link_name = jp.second.child_ ;
        NodePtr jnode = jp.second.node_ ;

        if ( parent_link_name.empty() || child_link_name.empty() ) return false ;

        NodePtr parent_link, child_link ;

        auto pl_it = links_.find(parent_link_name) ;
        if ( pl_it == links_.end() ) return false ;
        else parent_link = pl_it->second ;

        auto cl_it = links_.find(child_link_name) ;
        if ( cl_it == links_.end() ) return false ;
        else child_link = cl_it->second ;

        NodePtr ctrl_node(new Node) ;
        ctrl_node->setName(jp.second.name_ + "_ctrl") ;

        jnode->addChild(ctrl_node) ;

        parent_link->addChild(jnode) ;

        ctrl_node->addChild(child_link) ;

        jp.second.node_ = ctrl_node ;

        parent_link_tree[child_link_name] = parent_link_name ;
    }

    // find root

    for( const auto &lp: links_ ) {
        auto it = parent_link_tree.find(lp.first) ;
        if ( it == parent_link_tree.end() ) {
            root_node_ = lp.second ;
            break ;
        }

    }

    return true ;
}

RobotScenePtr URDFLoader::exportScene() {
    RobotScenePtr scene(new RobotScene) ;

    if ( !buildTree() ) return nullptr ;

    scene->addChild(root_node_) ;

    // create dofs

    map<string, JointNodePtr> mimic_joints ;

    for( const auto &jp: joints_ ) {
        const Joint &j = jp.second ;

        JointNodePtr jnode ;

        if ( j.type_ == "revolute" ) {
            RevoluteJoint *rj = new RevoluteJoint ;
            rj->lower_limit_ = j.lower_ ;
            rj->upper_limit_ = j.upper_ ;
            rj->axis_ = j.axis_ ;
            rj->node_ = j.node_ ;
            jnode.reset(rj) ;
        }
        else continue ;

        if ( j.mimic_joint_.empty() )
            scene->joints_.emplace(j.name_, jnode) ;
        else
            mimic_joints[j.name_] = jnode ;
    }

    for( const auto &jp: joints_ ) {
        const Joint &j = jp.second ;

        if ( j.mimic_joint_.empty() ) continue ;

        auto it = scene->joints_.find(j.mimic_joint_) ;
        if ( it == scene->joints_.end() ) continue ;

        auto jit = mimic_joints.find(j.name_) ;
        if ( jit != mimic_joints.end() ) {
            JointNodePtr dof = it->second ;
            dof->multipliers_.push_back(j.mimic_multiplier_) ;
            dof->offsets_.push_back(j.mimic_offset_) ;
            dof->dependent_.push_back(jit->second) ;
        }
    }

    return scene ;
}

Isometry3f URDFLoader::parseOrigin(const xml_node &node) {

    string xyz = node.attribute("xyz").as_string() ;
    string rpy = node.attribute("rpy").as_string() ;

    Isometry3f tr ;
    tr.setIdentity() ;

    if ( !xyz.empty() ) {
        Vector3f t = parse_vec3(xyz) ;
        tr.translate(t) ;
    }

    if ( !rpy.empty() ) {
        Vector3f r = parse_vec3(rpy) ;
        Quaternionf q ;

        q = AngleAxisf(r.z(), Vector3f::UnitZ()) * AngleAxisf(r.y(), Vector3f::UnitY()) * AngleAxisf(r.x(), Vector3f::UnitX());

        tr.rotate(q) ;
    }

    return tr ;
}

bool URDFLoader::resolveUri(const std::string &uri, std::string &path) {

    if ( startsWith(uri, "package://") ) {
        size_t pos = uri.find_first_of('/', 10) ;
        if ( pos == string::npos ) return false ;
        string package_str = uri.substr(10, pos-10) ;
        string package_subpath = uri.substr(pos+1) ;

        auto it = package_map_.find(package_str) ;
        if ( it == package_map_.end() ) return false ;
        path = Path(it->second, package_subpath).toString() ;
        return true ;
    }

    return false;

}

NodePtr URDFLoader::parseGeometry(const xml_node &node, const MaterialPtr &mat, Vector3f &sc) {

    if ( xml_node mesh_node = node.child("mesh") ) {
        NodePtr geom(new Node) ;

        string uri = mesh_node.attribute("filename").as_string(), path ;

        geom->setName(uri) ;

        if ( !resolveUri(uri, path) ) return nullptr ;

        string scale = mesh_node.attribute("scale").as_string() ;

        if ( !scale.empty() ) {
            sc = parse_vec3(scale) ;
        }

        ScenePtr scene(new Scene) ;
        scene->load(path, geom) ;

        // replace all materials in loaded model with that provided in urdf
        if ( mat ) {
            geom->visit([&](Node &n) {
                for( auto &dr: n.drawables() ) {
                    dr->setMaterial(mat) ;
                }
            }) ;
        }

        return geom ;
    } else if ( xml_node box_node = node.child("box") ) {
        string sz = box_node.attribute("size").as_string() ;
        Vector3f hs = parse_vec3(sz)/2 ;

        NodePtr geom_node(new Node) ;

        GeometryPtr geom(new BoxGeometry(hs)) ;

        DrawablePtr dr(new Drawable(geom, mat)) ;

        geom_node->addDrawable(dr) ;

        return geom_node ;
    } else if ( xml_node cylinder_node = node.child("cylinder") ) {

        float radius = cylinder_node.attribute("radius").as_float(0) ;
        float length = cylinder_node.attribute("length").as_float(0) ;

        NodePtr geom_node(new Node) ;

        GeometryPtr geom(new CylinderGeometry(radius, length)) ;

        DrawablePtr dr(new Drawable(geom, mat)) ;

        geom_node->addDrawable(dr) ;

        return geom_node ;
    } else if ( xml_node cylinder_node = node.child("sphere") ) {

        float radius = cylinder_node.attribute("radius").as_float(0) ;

        NodePtr geom_node(new Node) ;

        GeometryPtr geom(new SphereGeometry(radius)) ;

        DrawablePtr dr(new Drawable(geom, mat)) ;

        geom_node->addDrawable(dr) ;

        return geom_node ;
    }

    return nullptr ;
}

void URDFLoader::parseMaterial(const xml_node &node)
{
    string name = node.attribute("name").as_string() ;
    if ( name.empty() ) return ;

    if ( xml_node clr_node = node.child("color") ) {
        string rgba = clr_node.attribute("rgba").as_string() ;
        if ( !rgba.empty() ) {
            Vector4f clr = parse_vec4(rgba) ;
            PhongMaterial *mat = new PhongMaterial ;
            mat->setShininess(0);
            mat->setSpecular(Vector4f(0, 0, 0, 1)) ;
            mat->setDiffuse(clr) ;

            materials_.emplace(name, MaterialPtr(mat)) ;
            return ;
        }
    }

    if ( xml_node texture_node = node.child("texture") ) {
        string uri = texture_node.attribute("filename").as_string(), path ;

        if ( resolveUri(uri, path) ) {

            Texture2D s ;
            s.image_url_ = path ;

            DiffuseMapMaterial *mat = new DiffuseMapMaterial ;
            mat->setShininess(0);
            mat->setSpecular(Vector4f(0, 0, 0, 1)) ;
            mat->setDiffuse(s) ;

            materials_.emplace(name, MaterialPtr(mat)) ;
            return ;
        }
    }


}

}}


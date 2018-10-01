#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/scene/mesh.hpp>
#include <cvx/viz/scene/node.hpp>
#include <cvx/viz/scene/drawable.hpp>
#include <cvx/viz/scene/material.hpp>
#include <cvx/viz/scene/camera.hpp>

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace viz {

Scene::Scene() {}

class VertexAccumulator: public ConstNodeVisitor {
public:
    VertexAccumulator() = default;

    void visit(const Node &node) override {
        Isometry3f tf = node.globalTransform() ;
        for( const DrawablePtr &dr: node.drawables() ) {
            MeshPtr mesh = std::dynamic_pointer_cast<Mesh>(dr->geometry()) ;
            if ( mesh ) {
                for( const Vector3f &v: mesh->vertices().data() ) {
                    Vector3f p = tf * v ;
                    accum(p) ;
                }
            }
        }
        visitChildren(node) ;
    }

    virtual void accum(const Vector3f &v) = 0 ;
};

class GeometryCenterVisitor: public VertexAccumulator {
public:

    void accum(const Vector3f &v) {
        count_ ++ ;
        center_ += v ;
    }

    Vector3f center() const { return center_ / count_ ; }

    Vector3f center_ {0, 0, 0} ;
    uint count_ = 0 ;
};

class GeometryRadiusVisitor: public VertexAccumulator {
public:

    GeometryRadiusVisitor(const Vector3f &center): center_(center) {}
    void accum(const Vector3f &v) {
        float dist = (v - center_).squaredNorm() ;
        max_dist_ = std::max(max_dist_, dist) ;
    }

    float radius() const { return sqrt(max_dist_) ; }

    float max_dist_ = 0 ;
    Vector3f center_ ;
};

Vector3f Scene::geomCenter() const
{
    GeometryCenterVisitor v ;
    v.visit(*this) ;
    return v.center() ;
}

float Scene::geomRadius(const Vector3f &center) const
{
    GeometryRadiusVisitor v(center) ;
    v.visit(*this) ;
    return v.radius() ;

}

class NodeFinder: public NodeVisitor {
public:
    NodeFinder(const string &name): name_(name) {}
    void visit(Node &node) override {
        if ( node.name() == name_ ) {
            node_ = node.shared_from_this() ;
        }
        else
            visitChildren(node) ;
    }

    string name_ ;
    NodePtr node_ ;
} ;

NodePtr Node::findNodeByName(const std::string &name)
{
    NodeFinder nf(name) ;
    nf.visit(*this) ;
    return nf.node_ ;
}

bool Node::hit(const Ray &ray, Hit &hit)
{
    if ( !is_visible_ ) return false ;

    Isometry3f tf = globalTransform().inverse() ;
    for( const DrawablePtr &dr: drawables() ) {
        GeometryPtr geom = dr->geometry() ;
        float t ;
        Ray tr(ray, tf) ; // ray transform to local coordinate system
        if ( geom->intersect(tr,  t) && t < hit.t_ ) {
            hit.geom_ = geom.get() ;
            hit.node_ = this ;
            hit.t_ = t ;
        }
    }

    for( auto &c: children() ) {
        c->hit(ray, hit) ;
    }

    return hit.geom_ != nullptr ;
}




}}

#ifndef __VSIM_NODE_HPP__
#define __VSIM_NODE_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <functional>

#include <Eigen/Geometry>

#include <cvx/viz/scene/scene_fwd.hpp>
#include <cvx/viz/scene/drawable.hpp>

namespace cvx {

class NodeVisitor ;

// a hieracrchy of nodes. each node applies a transformation to the attached geometries, cameras, lights

class Node: public std::enable_shared_from_this<Node> {
public:

    Node() { mat_.setIdentity() ; }

    Eigen::Isometry3f &matrix() { return mat_ ; }

    std::string name() const { return name_ ; }

    void setName(const std::string &name) { name_ = name ; }

    void addDrawable(const DrawablePtr &d) { drawables_.push_back(d) ;  }

    void addChild(const NodePtr &n) {
        children_.push_back(n) ;
        n->parent_ = this ;
    }

    const std::vector<DrawablePtr> &drawables() const { return drawables_ ; }
    const std::vector<NodePtr> &children() const { return children_ ; }

    size_t numDrawables() const { return drawables_.size() ; }

    DrawablePtr getDrawable(size_t idx) const {
        assert( idx < drawables_.size() ) ;
        return drawables_[idx] ;
    }

    size_t numChildren() const { return children_.size() ; }

    NodePtr getChild(size_t idx) const {
        assert( idx < children_.size() ) ;
        return children_[idx] ;
    }

    void addLight(const LightPtr &light) { lights_.push_back(light) ; }

    const std::vector<LightPtr> &lights() const { return lights_ ; }

    Eigen::Isometry3f globalTransform() const {
        if ( parent_ ) return parent_->globalTransform() * mat_ ;
        else return mat_ ;
    }

    void setVisible(bool visible) { is_visible_ = visible ; }

    bool isVisible() const { return is_visible_ ; }

    void visit(const std::function<void (Node &)> &f) {
        f(*this) ;
        for( auto &n: children_ )
            n->visit(f) ;
    }

    NodePtr findNodeByName(const std::string &name) ;

    // create a node that contains a reference to a light and no geometry

    NodePtr addLightNode(const LightPtr &light) {
        NodePtr n(new Node) ;
        n->addLight(light) ;
        addChild(n) ;
        return n ;
    }

    NodePtr addSimpleShapeNode(const GeometryPtr &geom, const MaterialPtr &mat) {
        NodePtr n(new Node) ;
        DrawablePtr dr(new Drawable(geom, mat)) ;
        n->addDrawable(dr) ;
        addChild(n) ;
        return n ;
    }

private:

    bool is_visible_ = true ;

    std::string name_ ;

    Eigen::Isometry3f mat_ ;             // transformation matrix to apply to child nodes and attached geometries

    std::vector<NodePtr> children_ ;      // child nodes
    std::vector<DrawablePtr> drawables_ ; // meshes associated with this node
    std::vector<LightPtr> lights_ ;

    Node *parent_ = nullptr;
};

class NodeVisitor {
public:
    NodeVisitor() = default ;

    virtual void visit(Node &node) = 0 ;

    void visitChildren(Node &n) {
        for( auto &c: n.children() ) {
            visit(*c) ;
        }
    }
};

class ConstNodeVisitor {
public:
    ConstNodeVisitor() = default ;

    virtual void visit(const Node &node) = 0 ;

    void visitChildren(const Node &n) {
        for( auto c: n.children() ) {
            visit(*c) ;
        }
    }
};


} // namespace cvx
#endif


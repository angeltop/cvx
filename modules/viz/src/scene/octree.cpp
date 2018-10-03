#include "octree.hpp"

using Eigen::Vector3f ;
using std::vector ;


namespace cvx { namespace viz { namespace detail {

OctreeNode::~OctreeNode()
{
   for ( int i = 0; i < 8; i++ )
       if (children_[i])
            delete children_[i];
}

const Vector3f Octree::offsets_[8] = { { -0.5, 0.5, -0.5 }, {0.5, 0.5, -0.5}, { -0.5, -0.5, -0.5 }, { 0.5, -0.5, -0.5 },
                                       { -0.5, 0.5, 0.5 }, {0.5, 0.5, 0.5}, { -0.5, -0.5, 0.5 }, { 0.5, -0.5, 0.5 } } ;

void Octree::getTriangleVertices(uint tindex, Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f &v2) {
    uint i0 = indices_[tindex] ;
    uint i1 = indices_[tindex+1] ;
    uint i2 = indices_[tindex+2] ;
    v0 = vertices_[i0] ;
    v1 = vertices_[i1] ;
    v2 = vertices_[i2] ;
}

Octree::Octree(const Mesh &mesh, uint max_depth, uint max_count):
    vertices_(mesh.vertices().data()), indices_(mesh.vertices().indices()),
    max_depth_(max_depth), max_count_(max_count), root_(new OctreeNode()) {

    Vector3f bmin, bmax ;
    mesh.computeBoundingBox(bmin, bmax) ;

    // to handle planar objects (i,e one dimension is zero)
    Vector3f sz = bmax - bmin ;
    float side = std::max(sz.x(), std::max(sz.y(), sz.z()) ) ;
    float hs = side/2.0f ;

    center_ = (bmin + bmax)/2.0 ;
    hs_ = Vector3f( hs, hs, hs ) ;

    for( uint i=0 ; i<indices_.size() ; i+=3 ) {
        Vector3f v0, v1, v2 ;

        getTriangleVertices(i, v0, v1, v2) ;

        insert(i, v0, v1, v2) ;
    }

}

void Octree::insertTriangle(OctreeNode *node, const Eigen::Vector3f &center, const Eigen::Vector3f &hs, uint depth, uint32_t tindex,
                            const Eigen::Vector3f &v0, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {

    if ( node->is_leaf_ ) {
        if ( node->data_.size() < max_count_ || depth == max_depth_ ) { // we have reach deepest layer, just store triangle index
            node->data_.push_back(tindex) ;
        } else { //
            node->is_leaf_ = false; // make this node internal so that when we reinsert data it will spread to child nodes
            for( uint32_t idx: node->data_ ) {
                Vector3f v0, v1, v2 ;
                getTriangleVertices(idx, v0, v1, v2) ;
                insertTriangle(node, center, hs, depth, idx, v0, v1, v2) ;
            }
            node->data_.clear() ;
            insertTriangle(node, center, hs, depth, tindex, v0, v1, v2) ;
        }
    } else {
        for( uint i=0 ; i<8 ; i++ ) {
            Vector3f child_center  = center  + Vector3f((offsets_[i].array() * hs.array())) ;

      /*      if ( triangleInsideBox(v0, v1, v2, child_center, hs/2 ) ||
                 triangleIntersectsBox(v0, v1, v2, child_center, hs/2 ) ) {
*/
            if ( !triangleOutsideBox(v0, v1, v2, child_center, hs/2)) { // we use a cheap test here
                if ( node->children_[i] == nullptr )
                    node->children_[i] = new OctreeNode();    // the child intersects the triangle so push it there

                insertTriangle(node->children_[i], child_center, hs/2, depth + 1,  tindex, v0, v1, v2) ;
            }
        }
    }
}


void Octree::insert(uint32_t tindex, const Eigen::Vector3f &v0, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
    insertTriangle(root_, center_, hs_, 0,  tindex, v0, v1, v2) ;
}

bool Octree::intersect(OctreeNode *node, const Ray &r, const Eigen::Vector3f &center, const Eigen::Vector3f &hs, uint &tindex, float &mint) {

    // first cheap test with octree node AABB

    float t ;
    AABB box(center - hs, center + hs) ;

    if ( !rayIntersectsAABB(r, box, t) ) return false ;  // no need to go deeper

    if ( node->is_leaf_ ) { // reached a leaf node, check ray/triangle intersections

        for( uint idx: node->data_ ) {

            Vector3f v0, v1, v2 ;
            getTriangleVertices(idx, v0, v1, v2) ;

            if ( rayIntersectsTriangle(r, v0, v1, v2, true, t) ) {
                if ( t < mint ) {
                    mint = t ;
                    tindex = t ;
                }

                return true ;

            }
        }

    } else { // recurse on child nodes

        for( uint i=0 ; i<8 ; i++ ) {
            if ( node->children_[i] != nullptr ) {
                Vector3f child_center  = center  + Vector3f((offsets_[i].array() * hs.array())) ;
                intersect(node->children_[i], r, child_center, hs/2, tindex, mint ) ;
            }
        }
    }

    return false ;

}

bool Octree::intersect(const Ray &ray, uint &tindex, float &t) {
    t = std::numeric_limits<float>::max() ;
    return intersect(root_, ray, center_, hs_, tindex, t) ;
}


}}}

#ifndef __CVX_VIZ_OCTREE_HPP__
#define __CVX_VIZ_OCTREE_HPP__

#include <Eigen/Core>
#include <vector>

#include <cvx/viz/scene/mesh.hpp>

#include "intersect.hpp"

using Eigen::Vector3f ;
using std::vector ;


namespace cvx { namespace viz { namespace detail {

struct OctreeNode
{
    vector<uint64_t> data_;
    OctreeNode* children_[8];
    uint count_ = 0 ;
    bool is_leaf_ = false ;

    OctreeNode()
    {
        for (int i = 0; i < 8; i++)
           children_[i] = nullptr;
    }

    ~OctreeNode()
    {
        for (int i = 0; i < 8; i++)
            if (children_[i])
                delete children_[i];
    }



};
class Octree
{
protected:

    const Vector3f offsets_[8] = { { -0.5, 0.5, -0.5 }, {0.5, 0.5, -0.5}, { -0.5, -0.5, -0.5 }, { 0.5, -0.5, -0.5 },
                                         { -0.5, 0.5, 0.5 }, {0.5, 0.5, 0.5}, { -0.5, -0.5, 0.5 }, { 0.5, -0.5, 0.5 } } ;

    Vector3f bmin_, bmax_, cell_ ;
    OctreeNode* root_;

public:
    // create octree with given bounds and cell size

    Octree( const Mesh *mesh, const Vector3f &bmin, const Vector3f &bmax, uint max_depth):
        bmin_(bmin), bmax_(bmax), mesh_(mesh), max_depth_(max_depth), root_(nullptr) {

        auto &&vertices = mesh->vertices().data() ;
        assert( !vertices.empty() ) ;

        auto &&indices = mesh->vertices().indices() ;

        for( uint i=0 ; i<indices.size() ; i+=3 ) {
            uint i0 = indices[i] ;
            uint i1 = indices[i+1] ;
            uint i2 = indices[i+2] ;
            const Vector3f &v0 = vertices[i0] ;
            const Vector3f &v1 = vertices[i1] ;
            const Vector3f &v2 = vertices[i2] ;

            insert(i, v0, v1, v2) ;
        }


    }
    ~Octree() { delete root_; }


    const uint max_count = 100 ;

    uint insertTriangle( OctreeNode *node, const Vector3f &center, const Vector3f &hs, uint depth, uint count,
                         uint64_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2) {


        if ( depth < max_depth_  ) {
            node->count_ ++ ;
            for( uint i=0 ; i<8 ; i++ ) {
                Vector3f child_center  = center  + Vector3f((offsets_[i].array() * hs.array())) ;

             //   std::cout <<  (child_center-hs/2).adjoint() << ' ' << (child_center+hs/2).adjoint() << std::endl ;
                if ( triangleInsideBox(v0, v1, v2, child_center, hs/2 ) ||
                     triangleIntersectsBox(v0, v1, v2, child_center, hs/2 ) ) {

             //       std::cout << depth << ' ' << i << std::endl ;
                    if ( node->children_[i] == nullptr )
                        node->children_[i] = new OctreeNode();

                    insertTriangle(node->children_[i], child_center, hs/2, depth + 1, count, tindex, v0, v1, v2) ;
                }
            }
        }
        else {
            node->is_leaf_ = true ;
            node->data_.push_back(tindex) ;
        }

        return 0 ;
    }

    // insert new point and associated data
    void insert(uint64_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2)
    {

        if ( !root_ ) root_ = new OctreeNode();

        Vector3f hs = (bmax_ - bmin_)/2.0 ;

      //  std::cout << v0.adjoint() << ' ' << v1.adjoint() << ' ' << v2.adjoint() << std::endl ;
        insertTriangle(root_, bmin_ + hs, hs, 0, 0, tindex, v0, v1, v2) ;

        std::cout << tindex << std::endl ;
    }

    bool intersect(OctreeNode *node, const Ray &r, const Vector3f &center, const Vector3f &hs, uint &tindex, float &mint) {
        AABB box(center - hs, center + hs) ;
        float t ;
        if ( !rayIntersectsAABB(r, box, t) ) return false ;

        auto &&vertices = mesh_->vertices().data() ;
        assert( !vertices.empty() ) ;

        auto &&indices = mesh_->vertices().indices() ;

        if ( node->is_leaf_ ) {

            for( uint idx: node->data_ ) {
                uint i0 = indices[idx] ;
                uint i1 = indices[idx+1] ;
                uint i2 = indices[idx+2] ;
                Eigen::Vector3f v0 = vertices[i0] ;
                Eigen::Vector3f v1 = vertices[i1] ;
                Eigen::Vector3f v2 = vertices[i2] ;

                if ( rayIntersectsTriangle(r, v0, v1, v2, true, t) ) {
                    if ( t < mint ) {
                        mint = t ;
                        tindex = t ;
                    }

                    return true ;

                }
            }

        } else {

            for( uint i=0 ; i<8 ; i++ ) {
                if ( node->children_[i] != nullptr ) {
                    Vector3f child_center  = center  + Vector3f((offsets_[i].array() * hs.array())) ;
                    intersect(node->children_[i], r, child_center, hs/2, tindex, mint ) ;
                }
            }
        }

        return false ;

    }

    bool intersectRay(const Ray &ray, uint &tindex, float &t) {
        Vector3f hs = (bmax_ - bmin_)/2.0 ;
        t = std::numeric_limits<float>::max() ;
        return intersect(root_, ray, bmin_ + hs, hs, tindex, t) ;
    }

    uint max_depth_ ;
    const Mesh *mesh_ ;

};

}}}

#endif

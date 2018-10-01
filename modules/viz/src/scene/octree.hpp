#ifndef __CVX_VIZ_OCTREE_HPP__
#define __CVX_VIZ_OCTREE_HPP__

#include <Eigen/Core>
#include <vector>

#include "intersect.hpp"

using Eigen::Vector3f ;
using std::vector ;


namespace cvx { namespace viz { namespace detail {

struct OctreeNode
{
    vector<uint64_t> data_;
    OctreeNode* children_[8];
    uint count_ = 0 ;

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

    Octree( const Vector3f &bmin, const Vector3f &bmax, const Vector3f &cell): bmin_(bmin), bmax_(bmax), cell_(cell), root_(nullptr) {}
    virtual ~Octree() { delete root_; }

    const uint max_depth = 10 ;
    const uint max_count = 100 ;

    uint insertTriangle( OctreeNode *node, const Vector3f &center, const Vector3f &hs, uint depth, uint count,
                         uint64_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2) {


        if ( depth < max_depth  && count < max_count ) {
            for( uint i=0 ; i<8 ; i++ ) {
                Vector3f child_center  = center  + Vector3f((offsets_[i].array() * hs.array())) ;
                if ( triangleIntersectsBox(v0, v1, v2, child_center, hs/2 ) ) {
                    if ( node->children_[i] == nullptr )
                        node->children_[i] = new OctreeNode();

                    insertTriangle(node->children_[i], child_center, hs/2, depth + 1, count, tindex, v0, v1, v2) ;
                }

            }
        }
        else {
            if ( triangleIntersectsBox(v0, v1, v2, center, hs ) ) {
                node->data_.push_back(tindex) ;
            }
        }
    }

    // insert new point and associated data
    void insert(uint64_t tindex, const Vector3f &v0, const Vector3f &v1, const Vector3f &v2)
    {

        if ( !root_ ) root_ = new OctreeNode();

        Vector3f hs = (bmax_ - bmin_)/2.0 ;

        insertTriangle(root_, bmin_ + hs, hs, 0, 0, tindex, v0, v1, v2) ;
    }



};

}}}

#endif

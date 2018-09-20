#ifndef __OCTREE_HPP__
#define __OCTREE_HPP__

#include <Eigen/Core>
#include <vector>
#include <functional>

#include <cvx/util/geometry/point_list.hpp>

using Eigen::Vector3f ;
using std::vector ;

#define COMPUTE_SIDE(i, bit, p, mid, newMin, newMax) \
if (p >= mid) { i |= bit;  newMin = mid; }\
else newMax = mid;

inline bool operator >= (const Vector3f &a, const Vector3f &b) {
    return (a.x() >= b.x() && a.y() >= b.y() && a.z() >= b.z()) ;
}

inline bool operator < (const Vector3f &a, const Vector3f &b) {
    return (a.x() < b.x() && a.y() < b.y() && a.z() < b.z()) ;
}

namespace cvx { namespace util {

// Octree data structure parameterized by datatype U i.e. data representation for each point (it can be an index to an external container )
template<typename U>
class Octree
{
protected:

    struct Node
    {
        vector<U> data_;
        Node* children_[8];
        Node()
        {
            for (int i = 0; i < 8; i++)
               children_[i] = 0;
        }
        virtual ~Node()
        {
            for (int i = 0; i < 8; i++)
                if (children_[i])
                    delete children_[i];
        }
    };

    Vector3f bmin_, bmax_, cell_ ;
    Node* root_;

public:
    // create octree with given bounds and cell size

    Octree( const Vector3f &bmin, const Vector3f &bmax, const Vector3f &cell): bmin_(bmin), bmax_(bmax), cell_(cell), root_(0) {}
    virtual ~Octree() { delete root_; }

    // insert new point and associated data
    void insert(const Vector3f &pos, const U &data)
    {
        Vector3f ppos(pos);

        assert(ppos >= bmin_ && ppos < bmax_);

        Vector3f currMin(bmin_);
        Vector3f currMax(bmax_);
        Vector3f delta = bmax_ - bmin_;

        if ( !root_ ) root_ = new Node();

        Node* currNode = root_ ;

        while (delta >= cell_)
        {
            Vector3f mid = (delta * 0.5f) + currMin ;
            Vector3f newMin(currMin);
            Vector3f newMax(currMax);
            int index = 0;
            COMPUTE_SIDE(index, 1, ppos.x(), mid.x(), newMin.x(), newMax.x())
            COMPUTE_SIDE(index, 2, ppos.y(), mid.y(), newMin.y(), newMax.y())
            COMPUTE_SIDE(index, 4, ppos.z(), mid.z(), newMin.z(), newMax.z())
            if (!(currNode->children_[index]))
                currNode->children_[index] = new Node();
            currNode = currNode->children_[index];
            currMin = newMin;
            currMax = newMax;
            delta = currMax - currMin;
        }

        currNode->data_.push_back(data) ;
    }

    // Recursively visit octree nodes. Traversal finished if callback function returns false
    void traverse(const std::function<bool (const Vector3f &, const Vector3f &, const vector<U> &)> &callback) {
        traverseRecursive(callback, bmin_, bmax_, root_);
    }

private:

    void traverseRecursive(const std::function<bool (const Vector3f &, const Vector3f &, const vector<U> &)> &callback,
                           const Vector3f& currMin, const Vector3f& currMax,
                           Node* currNode)  {
          if ( !currNode || !callback(currMin, currMax, currNode->data_) ) return;

          Vector3f delta = currMax - currMin;
          Vector3f mid = (delta * 0.5f) + currMin;
          traverseRecursive(callback, currMin, mid, currNode->children_[0]);
          traverseRecursive(callback, Vector3f(mid.x(), currMin.y(), currMin.z()), Vector3f(currMax.x(), mid.y(), mid.z()), currNode->children_[1]);
          traverseRecursive(callback, Vector3f(currMin.x(), mid.y(), currMin.z()), Vector3f(mid.x(), currMax.y(), mid.z()), currNode->children_[2]);
          traverseRecursive(callback, Vector3f(mid.x(), mid.y(), currMin.z()), Vector3f(currMax.x(), currMax.y(), mid.z()), currNode->children_[3]);
          traverseRecursive(callback, Vector3f(currMin.x(), currMin.y(), mid.z()), Vector3f(mid.x(), mid.y(), currMax.z()), currNode->children_[4]);
          traverseRecursive(callback, Vector3f(mid.x(), currMin.y(), mid.z()), Vector3f(currMax.x(), mid.y(), currMax.z()), currNode->children_[5]);
          traverseRecursive(callback, Vector3f(currMin.x(), mid.y(), mid.z()), Vector3f(mid.x(), currMax.y(), currMax.z()), currNode->children_[6]);
          traverseRecursive(callback, mid, currMax, currNode->children_[7]);
      }

};

typedef Octree<Vector3f> OctreeCloud ; // points are stored inside the nodes
typedef Octree<uint> OctreeIndexed ;   // indexes to data stored ;

// Subsampling of pointcloud be means of octree. The average of points inside each leaf node is computed and then the point closest to it is selected.
void sampleCloudCenters(const EPointList3f &cloud, float min_cell_size_, EPointList3f &res, const Vector3f &rmin, const Vector3f &rmax) ;

}}

#endif

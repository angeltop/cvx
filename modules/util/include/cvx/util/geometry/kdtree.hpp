#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__

#include <vector>
#include <memory>
#include <Eigen/Core>

#include <cvx/util/geometry/point_list.hpp>

// 2D/3D point cloud Search (wrapper over nanoflann)

namespace cvx {

class KDTreeIndex3 ;

class KDTree3
{
public:

    typedef Eigen::Vector3f point_t ;
    typedef EPointList3f point_list_t ;

    KDTree3() {}
    KDTree3(const point_list_t &data) ;

    void train(const point_list_t &data) ;

    // nearest point
    uint nearest(const point_t &q) ;
    uint nearest(const point_t &q, float &dist) ;

    void knearest(const point_t &q, uint k, std::vector<uint> &indexes) ;
    void knearest(const point_t &q, uint k, std::vector<uint> &indexes, std::vector<float> &distances) ;

    void withinRadius(const point_t &q, float radius, std::vector<uint> &indexes) ;
    void withinRadius(const point_t &q, float radius, std::vector<uint> &indexes, std::vector<float> &distances) ;

private:

    std::shared_ptr<KDTreeIndex3> index_ ;
} ;

class KDTreeIndex2 ;

class KDTree2
{
public:
    typedef Eigen::Vector2f point_t ;
    typedef PointList<float, 2> point_list_t ;

    KDTree2() {}
    KDTree2(const point_list_t &data) ;

    void train(const point_list_t &data) ;

    // nearest point
    uint nearest(const point_t &q) ;
    uint nearest(const point_t &q, float &dist) ;

    void knearest(const point_t &q, uint k, std::vector<uint> &indexes) ;
    void knearest(const point_t &q, uint k, std::vector<uint> &indexes, std::vector<float> &distances) ;

    void withinRadius(const point_t &q, float radius, std::vector<uint> &indexes) ;
    void withinRadius(const point_t &q, float radius, std::vector<uint> &indexes, std::vector<float> &distances) ;

private:

    std::shared_ptr<KDTreeIndex2> index_ ;
} ;

}

#endif

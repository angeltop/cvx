#ifndef __POINT_LIST_HPP__
#define __POINT_LIST_HPP__

#include <cvx/util/geometry/point.hpp>

// https://stackoverflow.com/questions/24311357/why-are-initializer-lists-not-available-when-changing-the-allocator-of-stdvect/24346064
// #include <Eigen/StdVector>

namespace cvx { namespace util {

template <typename T, int D>
using pl_point_t = Eigen::Matrix<T, D, 1> ;

template <typename T, int D, typename alloc>
using pl_container_t = std::vector<pl_point_t<T, D>, alloc> ;



template <class T, int D, typename alloc = std::allocator<T>>
class PointList: public pl_container_t<T, D, alloc>
{
    using Point = pl_point_t<T, D> ;
    using Base = pl_container_t<T, D, alloc> ;
    using Map = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor>>;
    using ConstMap = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor>>;


public:

    PointList(): Base() {}

    PointList(uint n):
       Base(n) {}

    PointList(const std::initializer_list<Point> &data) {
        this->resize(data.size()) ;
        std::copy(data.begin(), data.end(), this->begin()) ;

    }

    // fill with row major data i.e. (x, y) or column major ( x1, x2 ... y1, y2 ...)

    PointList(T *data, size_t n, bool row_major = true) {

        this->resize(n) ;
        if ( row_major )
            for (uint i=0 ; i<n ; i++, data += D ) (*this)[i] = *reinterpret_cast<Point *>(data) ;
        else {
            for( uint j=0 ; j<D ; j++ )
                for (uint i=0 ; i<n ; i++, data++ )
                    (*this)[i][j] = *data ;
        }
    }

    PointList(const PointList<T, D> &other): Base(other) {
    }

    PointList(const Eigen::Matrix<T, Eigen::Dynamic, D> &x): Base(x.rows()) {
        assert(x.cols() == D);
        for( uint idx = 0 ; idx<x.rows() ; idx++ )
            (*this)[idx] = x.row(idx) ;
    }


    PointList(const cv::Mat &src) {
         assert(src.cols == D);
         this->resize(src.rows) ;
         cv::Mat dst(src.rows, src.cols, cv::DataType<T>::type, &(*this)[0], D*(size_t)(sizeof(T)));
         src.convertTo(dst, dst.type());
         assert( dst.data == (uchar*)this->data());
    }

     Map asEigenMap() {
        return Map(reinterpret_cast<T *>(this->data()->data()), this->size(), D) ;
    }

     ConstMap asEigenMap() const{
        return ConstMap(reinterpret_cast<const T *>(this->data()->data()), this->size(), D) ;
    }

    Point center() const {
        return asEigenMap().colwise().mean();
    }

    void axes(double &l1, Point &v1, double &l2, Point &v2) const ;

    // Procrustes analysis
    // Find the transform  that aligns this shape with the other one:
    // this' = T(s) * T(theta) * this + T(tx, ty)
    Eigen::Affine2d align(const PointList<T, D> &other) const ;

    template <int Mode>
    void transform(const Eigen::Transform<T, D, Mode> &xf) {
        for(uint i=0 ;i <this->size() ; i++ ) (*this)[i] = xf * (*this)[i] ;

    }

    std::pair< Point, Point > bbox() const {
        auto a = asEigenMap() ;
        return std::make_pair(a.colwise().minCoeff(), a.colwise().maxCoeff()) ;
    }

    double norm() const { return asEigenMap().norm() ; }

    cv::Mat toCVMat() const {
         return cv::Mat(this->size(), 1, cv::DataType< cv::Vec<T, D> >::type, (void*)this->data(), D * sizeof(T));
    }
} ;

using PointList2f = PointList<float, 2> ;
using PointList3f= PointList<float, 3> ;
using PointList4f = PointList<float, 4, Eigen::aligned_allocator<Eigen::Vector4f>> ;

using PointList2d = PointList<double, 2, Eigen::aligned_allocator<Eigen::Vector2d>> ;
using PointList3d= PointList<double, 3> ;
using PointList4d = PointList<double, 4, Eigen::aligned_allocator<Eigen::Vector4d>> ;


}}


#endif

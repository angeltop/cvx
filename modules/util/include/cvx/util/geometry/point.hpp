#ifndef __Point_HPP__
#define __Point_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace cvx { namespace util {

template <class T, int D>
using Point = Eigen::Matrix<T, D, 1> ;

template<class T>
inline Point<T, 2> max(const Point<T, 2> &a, const Point<T, 2> &b) {
    return Point<T, 2>(std::max<T>(a.x(), b.x()), std::max<T>(a.y(), b.y())) ;
}

template<class T>
inline Point<T, 2> min(const Point<T, 2> &a, const Point<T, 2> &b) {
    return Point<T, 2>(std::min<T>(a.x(), b.x()), std::min<T>(a.y(), b.y())) ;
}

template<class T>
inline Point<T, 3> max(const Point<T, 3> &a, const Point<T, 3> &b) {
    return Point<T, 3>(std::max<T>(a.x(), b.x()), std::max<T>(a.y(), b.y()), std::max<T>(a.z(), b.z())) ;
}

template<class T>
inline Point<T, 3> min(const Point<T, 3> &a, const Point<T, 3> &b) {
    return Point<T, 3>(std::min<T>(a.x(), b.x()), std::min<T>(a.y(), b.y()), std::max<T>(a.z(), b.z())) ;
}

template<class T, int D>
inline Point<T, D> max(const Point<T, D> &a, const Point<T, D> &b, const Point<T, D> &c) {
    return max<T>(a, max<T>(b, c)) ;
}

template<class T, int D>
inline Point<T, D> min(const Point<T, D> &a, const Point<T, D> &b, const Point<T, D> &c) {
    return min<T>(a, min<T>(b, c)) ;
}

typedef Point<float, 2> Point2f ;
typedef Point<double, 2> Point2d ;

typedef Point<int, 2> Point2i ;

typedef Point<float, 3> Point3f ;
typedef Point<double, 3> Point3d ;

}}


#endif

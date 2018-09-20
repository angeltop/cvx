#ifndef __Point_HPP__
#define __Point_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace cvx {

template <class T, int D>
class Point: public Eigen::Matrix<T, 1, D> {
public:
    typedef Eigen::Matrix<T, 1, D> base_type ;

    Point() { this->setZero() ; }

    template <class Q>
    Point(Q x, Q y): base_type((T)x, (T)y) {}

    template <class Q>
    Point(Q x, Q y, Q z): base_type((T)x, (T)y, (T)z) {}

    template <class Q>
    Point(const cv::Point_<Q> &p): base_type((T)p.x, (T)p.y) {}

    template <class Q>
    Point(const cv::Point3_<Q> &p): base_type((T)p.x, (T)p.y, (T)p.z) {}

    template <class Q>
    operator cv::Point_<Q>() const { return cv::Point_<Q>(this->x(), this->y()) ; }

    template <class Q>
    operator cv::Point3_<Q>() const { return cv::Point_<Q>(this->x(), this->y(), this->z()) ; }

    template <class Q>
    operator Eigen::Matrix<Q, 1, D> () const { return Eigen::Matrix<Q, 1, D>(*this) ; }

    template<typename OtherDerived>
    Point(const Eigen::MatrixBase<OtherDerived>& other)
        : base_type(other)
    { }

    template<typename OtherDerived>
    Point & operator= (const Eigen::MatrixBase <OtherDerived>& other) {
        this->base_type::operator=(other);
        return *this;
    }

};

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

}


#endif

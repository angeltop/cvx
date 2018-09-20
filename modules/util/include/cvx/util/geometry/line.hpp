#ifndef __LINE_HPP__
#define __LINE_HPP__

#include <cvx/util/geometry/point.hpp>
#include <cvx/util/geometry/point_list.hpp>
#include <cvx/util/math/random.hpp>

#include <Eigen/Eigenvalues>

namespace cvx {

template<class T, int D>
class Line
{
public:

    typedef Point<T, D> point_t ;

    // Line passing from a point p with direction dir

    Line(const point_t &p, const point_t &dir): p_(p), d_(dir) {
        d_.normalize() ;
    }

    // Line in the form a x + b * y + c = 0 ;
    Line(T a, T b, T c) ;

    // Line in the form a[0] * x + a[1] * y + a[2] = 0 ;

    Line(T a[3])  ;

    // Get line direction vector

    point_t dir() const { return d_ ; }
    point_t origin() const { return p_ ; }

    void coeffs(T &a, T &b, T &c) const {
        a = d_.y() ; b = -d_.x() ;
        c = d_.x() * p_.y() - d_.y() * p_.x() ;
    }

    void coeffs(T a[3]) const {
        coeffs(a[0], a[1], a[2]) ;
    }

    // Find the intersection between  two lines

    bool intersection(const Line<T, D> &other, point_t &pp, T e = std::numeric_limits<T>::epsilon() ) const {

        point_t a = d_ ;
        point_t b = other.d_ ;
        point_t c = other.p_ - p_ ;
        point_t ab = a.cross(b) ;

        if ( fabs(c.dot(ab)) > e ) return false ;

        T det = ab.norm() ;

        // parallel lines
        if ( det < e ) return false ;

        T s = c.cross(b).dot(ab) ;

        pp = p_ + s * a/det/det  ;

        return true ;
    }

    bool isCoplanar(const Line<T, D> &other, T e = std::numeric_limits<T>::epsilon()) const
    {
        point_t a = d_ ;
        point_t b = other.d_ ;
        point_t c = other.p_ - p_ ;

        double det = fabs(c.dot(a.cross(b))) ;

        return ( det < e )  ;
    }


    bool isParallel(const Line<T, D> &other, T e = std::numeric_limits<T>::epsilon() ) const {
        return ( isCoplanar(other) && d_.cross(other.d).norm() < e) ;
    }

    // Find the distance of a point to the line. Optionally returns the closest point on the line
    T distanceToPoint(const point_t &q, point_t *psd = NULL) const {
        point_t xx = p_ - q ;

        T dist = d_.cross(xx).norm() ;

        if ( psd ) *psd = p_ - d_ * xx.dot(d_) ;

        return fabs(dist) ;
    }

    point_t p_ ;
    point_t d_ ;

private:

    void init(T a, T b, T c) {
        if ( fabs(a) < std::numeric_limits<T>::epsilon() )
        {
            assert(fabs(b) > std::numeric_limits<T>::epsilon()) ;
            p_.x() = 0.0 ;  p_.y() = -c/b ;
            d_.x() = 1.0 ;  d_.y() = 0.0 ;
        }
        else if ( fabs(b) < std::numeric_limits<T>::epsilon() )
        {
            assert(fabs(a) > std::numeric_limits<T>::epsilon() ) ;
            p_.x() = -c/a ;  p_.y() = 0.0 ;
            d_.x() = 0.0  ;  d_.y() = 1.0 ;
        }
        else
        {
            d_.x() = -b ; d_.y() = a ; d_.normalize() ;
            p_.x() = -c/a/2.0 ; p_.y() = -c/b/2.0 ;
        }
    }
} ;


typedef Line<double, 2> Line2d ;
typedef Line<float, 2> Line2f ;
typedef Line<double, 3> Line3d ;
typedef Line<float, 3> Line3f ;


// 2D line segment

template<class T, int D>
class LineSegment
{
public:

    typedef Point<T, D> point_t ;

    // Constructors

    // Line segment defined from  two points
    LineSegment(const point_t &p1, const point_t &p2): pa_(p1), pb_(p2) {
    }

    Line<T, D> getLine() const { return Line<T, D>(pa_, pb_ - pa_) ; }

    // Get line direction vector

    point_t getDir() const {
        point_t d = pb_ - pa_ ;
        d.normalize() ;
        return d ;
    }

    // Swap end-points

    void invert() {
        swap(pa_, pb_) ;
    }

    // Find the intersection between the segment and a line

    bool intersection(const Line<T, D> &other,  point_t &p, T e = std::numeric_limits<T>::epsilon()) const {
        Line<T, D> l(pa_, pb_ - pa_) ;

        if ( !l.intersection(other, p, e) ) return false ;
        else
        {
            // Check if intersection point is inside segment

            T s = (pa_ - p).dot(pb_ - p) ;

            if ( s > 0 ) return false ;
        }

        return true ;
    }

    // Find the intersection (if any) with a line segment

    bool intersection(const LineSegment<T, D> &other, point_t &p, T e = std::numeric_limits<T>::epsilon()) const {
        Line<T, D> l(pa_, pb_ - pa_) ;

        if ( !other.intersection(l, p, e) ) return false ;
        else
        {
            // Check if intersection point is inside segment

            T s = (pa_ - p).dot(pb_ - p) ;
            T t = (other.pa_ - p).dot(other.pb_ - p) ;

            if ( s > 0 || t > 0 ) return false ;
        }

        return true ;
    }

    bool isParallel(const Line<T, D> &other, T e = std::numeric_limits<T>::epsilon()) const {
        return getLine().isParallel(other, e) ;
    }
    bool isParallel(const LineSegment<T, D> &other, T e = std::numeric_limits<T>::epsilon()) const {
        return getLine().isParallel(other.getLine(), e) ;
    }

    bool isCollinear(const Line<T, D> &other, T e = std::numeric_limits<T>::epsilon()) const {
        if ( other.distanceToPoint(pa_) < e &&
             other.distanceToPoint(pb_) < e  ) return true ;
        else return false ;
    }
    bool isCollinear(const LineSegment<T, D> &other, T e = std::numeric_limits<T>::epsilon()) const {
        return isCollinear(other.getLine(), e) ;
    }

    // Find the distance of a point to the line.
    // Optionally returns the closest point on the line

    T distanceToPoint(const point_t &p, point_t *psd = NULL) const {
        return  getLine().distanceToPoint(p, psd) ;
    }

    // For points that lye on the line the function returns a flag indicating
    // whether the point is inside the line segment (0), before the first end-point
    // (-1) and after the second end-point (1)

    int contains(const point_t &pp) const {
        if ( (pp - pa_).dot(pb_ - pa_) < 0 ) return -1 ;
        else if ( (pp - pb_).dot(pb_ - pa_) > 0 ) return 1 ;
        else return 0 ;
    }

    point_t pa_, pb_ ;
} ;

typedef LineSegment<double, 2> LineSegment2d ;
typedef LineSegment<float, 2> LineSegment2f ;
typedef LineSegment<double, 3> LineSegment3d ;
typedef LineSegment<float, 3> LineSegment3f ;

}
#endif

#ifndef	__RECTANGLE_HPP__
#define	__RECTANGLE_HPP__

#include <cvx/util/geometry/point.hpp>
#include <opencv2/opencv.hpp>

#include <math.h>

namespace cvx { namespace util {

template< class T >
class Rectangle
{
    typedef Point<T, 2> point_t ;

protected:

    point_t tl_;		// top left corner (origin)
    point_t br_;		// bottom right corner (corner)

 public:

    Rectangle(T left=0, T top=0, T width=0, T height=0);
    Rectangle(const point_t&, const point_t&);
    Rectangle(const Rectangle&);
    Rectangle(const cv::Rect_<T> &r): tl_(r.tl()), br_(r.br()) {}

    point_t origin() const	{ return tl_; }
    point_t corner() const	{ return br_; }
    point_t topLeft() const	{ return tl_; }
    point_t topCenter() const	{ return point_t((br_.x()+tl_.x())/2,tl_.y()); }
    point_t topRight() const	{ return point_t(br_.x(),tl_.y()); }
    point_t rightCenter() const	{ return point_t(br_.x(),(br_.y()+tl_.y())/2); }
    point_t bottomRight() const	{ return br_; }
    point_t bottomCenter() const	{ return point_t((br_.x()+tl_.x())/2,br_.y()); }
    point_t bottomLeft() const	{ return point_t(tl_.x(),br_.y()); }
    point_t leftCenter() const	{ return point_t(tl_.x(),(br_.y()+tl_.y())/2); }
    point_t center() const	{ return point_t((br_.x()+tl_.x())/2,(br_.y()+tl_.y())/2); }
    point_t extent() const	{ return point_t(br_.x()-tl_.x(),br_.y()-tl_.y()); }
    double area() const		{ return width()*height(); }
    double width() const		{ return br_.x()-tl_.x() ; }
    double height() const		{ return br_.y()-tl_.y() ; }

    Rectangle operator&&(const Rectangle&) const;	// intersection
    Rectangle operator||(const Rectangle&) const;	// union

    void operator+=(const point_t&);			// translate
    void operator-=(const point_t&);

    bool contains(const point_t &) const;
    bool contains(const Rectangle &) const;

    bool intersects(const Rectangle &) const;

    void moveTo(const point_t &);

    template<class Q>
    friend std::ostream & operator << (std::ostream &strm, const Rectangle<Q> &r) ;
};

template<class T>
inline Rectangle<T>::Rectangle(T left, T top, T width, T height)
{
    tl_ = point_t(left, top);
    br_ = point_t(left + width, top + height);
}

template<class T>
inline Rectangle<T>::Rectangle(const point_t& o, const point_t& c)
{
    tl_ = o;
    br_ = c;
}

template<class T>
inline Rectangle<T>::Rectangle(const Rectangle& r) : tl_(r.tl_), br_(r.br_) {}

template<class T>
inline Rectangle<T> Rectangle<T>::operator&&(const Rectangle<T>& r) const
{
    return Rectangle(point_t(std::max(tl_.x(), r.tl_.x()), std::max(tl_.y(), r.tl_.y())),
        point_t(std::min(br_.x(), r.br_.x()), std::min(br_.y(), r.br_.y()))) ;
}

template<class T>
inline Rectangle<T> Rectangle<T>::operator||(const Rectangle<T>& r) const
{
    return Rectangle(point_t(std::min(tl_.x(), r.tl_.x()), std::min(tl_.y(), r.tl_.y())),
        point_t(std::max(br_.x(), r.br_.x()), std::max(br_.y(), r.br_.y()))) ;
}

template<class T>
inline void Rectangle<T>::operator+=(const point_t& p)
{
    tl_ += p;
    br_ += p;
}

template<class T>
inline void Rectangle<T>::operator-=(const point_t& p)
{
    tl_ -= p;
    br_ -= p;
}

template<class T>
inline bool Rectangle<T>::contains(const point_t& p) const
{
    return (tl_.x() <= p.x() && tl_.y() <= p.y() ) && (p.x() <= br_.x() && p.y() <= br_.y());
}

template<class T>
inline bool Rectangle<T>::contains(const Rectangle& r) const
{
    return ( contains(r.tl_) && contains(r.br_) );
}

template<class T>
inline bool Rectangle<T>::intersects(const Rectangle& r) const
{
    if ( std::max(tl_.x(), r.tl_.x()) < std::min(br_.x(), r.br_.x()) &&
        std::max(tl_.y(), r.tl_.y()) < std::min(br_.y(), r.br_.y()) ) return true ;
    return false;
}

template<class T>
inline void Rectangle<T>::moveTo(const point_t& p)
{
    br_ += p-tl_;
    tl_ = p;
}

template<class T>
inline std::ostream & operator << (std::ostream &strm, const Rectangle<T> &r)
{
    return strm << "{ " << r.topLeft() << " " << r.bottomRight() << " }" ;
}

typedef Rectangle<double> Rect2d ;
typedef Rectangle<float> Rect2f ;

}}

#endif

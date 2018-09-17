#ifndef __POINT_LIST_HPP__
#define __POINT_LIST_HPP__

#include <cvx/util/geometry/point.hpp>
#include <Eigen/StdVector>

namespace cvx { namespace util {

template <class T, int D>
class PointList
{
public:

    typedef Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor> matrix_t ;

    PointList(uint n): mat_(n, D) {}

    // fill with row major data i.e. (x, y) or column major ( x1, x2 ... y1, y2 ...)
    PointList(T *data, int n, bool row_major = true) {
        if ( row_major )
            mat_ = Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor> >(data, n, D) ;
        else
            mat_ = Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::ColMajor> >(data, n, D) ;
    }

    template<class Q>
    PointList(const std::vector< Q > &pts) {
        mat_.resize(pts.size(), D) ;
        for(uint i=0 ; i<pts.size() ; i++) mat_.row(i) = pts[i] ;
    }

    PointList(const matrix_t &x): mat_(x) {}

    PointList(const cv::Mat &src) {
         assert(src.cols == D);
         mat_.resize(src.rows, D) ;
         cv::Mat dst(src.rows, src.cols, cv::DataType<T>::type, mat_.data(), (size_t)(mat_.stride()*sizeof(T)));
         src.convertTo(dst, dst.type());
         assert( dst.data == (uchar*)mat_.data());
    }

    Point<T, D> operator [] (uint idx) const { return mat_.row(idx) ; }
    Eigen::Map< Eigen::Matrix<T, 1, D> > operator [] (uint idx) { return  Eigen::Map< Eigen::Matrix<T, 1, D> >(mat_.data() + idx * mat_.stride()); }

    Point<T, D> center() const {
        return mat_.colwise().mean();
    }

    size_t size() const { return mat_.rows() ; }

    void axes(double &l1, Point<T, D> &v1, double &l2, Point<T, D> &v2) const ;

    // Procrustes analysis
    // Find the transform  that aligns this shape with the other one:
    // this' = T(s) * T(theta) * this + T(tx, ty)
    Eigen::Affine2d align(const PointList<T, D> &other) const ;

    void transform(const Eigen::Affine2d &xf) {
        mat_ = xf.cast<float>() * mat_.transpose() ;
    }

    std::pair< Point<T, D>, Point<T, D> > bbox() const {
        return std::make_pair(mat_.colwise().minCoeff(), mat_.colwise().maxCoeff()) ;
    }

    double norm() const { return mat_.norm() ; }
    void translate(const Point<T, D> &offset) ;
    void scale(double s) ;

    Eigen::Matrix<T, Eigen::Dynamic, 1>  toVector(bool row_major = true) const {
        Eigen::Matrix<T, Eigen::Dynamic, 1> res(mat_.rows() * D) ;
        if ( row_major )
            Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::RowMajor> >(res.data(), mat_.rows(), D) = mat_ ;
        else
            Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, D, Eigen::ColMajor> >(res.data(), mat_.rows(), D) = mat_ ;
        return res ;
    }

    const matrix_t &mat() const { return mat_ ; }
    cv::Mat toCVMat() const {
         return cv::Mat(mat_.rows(), 1, cv::DataType< cv::Vec<T, D> >::type, (void*)mat_.data(), mat_.stride() * sizeof(T));
    }

protected:

    matrix_t mat_ ;
} ;

typedef PointList<double, 2> PointList2d ;
typedef PointList<float, 2> PointList2f ;
typedef PointList<double, 3> PointList3d ;
typedef PointList<float, 3> PointList3f ;

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > EPointList3f ;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > EPointList3d ;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > EPointList2f ;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > EPointList2d ;

}
}

#endif

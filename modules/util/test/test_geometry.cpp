#include <cvx/util/geometry/point_list.hpp>
#include<Eigen/StdVector>


using namespace std ;
using namespace cvx::util ;

template <typename T, int D>
using point_t = Eigen::Matrix<T, D, 1> ;

template <typename T, int D, typename alloc>
using container_t = std::vector<point_t<T, D>, alloc> ;

template <class T, int D, typename alloc = std::allocator<T>>
class PointList2: public container_t<T, D, alloc>
{
public:

    using Point = point_t<T, D> ;
    using Base = container_t<T, D, alloc> ;

    PointList2(uint n):
       Base(n) {}

    PointList2(const std::initializer_list<Point> &data) {
        this->resize(data.size()) ;
        std::copy(data.begin(), data.end(), this->begin()) ;

    }

    // fill with row major data i.e. (x, y) or column major ( x1, x2 ... y1, y2 ...)

    PointList2(T *data, size_t n, bool row_major = true) {

        this->resize(n) ;
        if ( row_major )
            for (uint i=0 ; i<n ; i++, data += D ) (*this)[i] = *reinterpret_cast<Point *>(data) ;
        else {
            for( uint j=0 ; j<D ; j++ )
                for (uint i=0 ; i<n ; i++, data++ )
                    (*this)[i][j] = *data ;
        }
    }

    PointList2(const PointList2<T, D> &other): Base(other) {
    }

    PointList2(const Eigen::Matrix<T, Eigen::Dynamic, D> &x): Base(x.rows()) {
        for( uint idx = 0 ; idx<x.rows() ; idx++ )
            (*this)[idx] = x.row(idx) ;
    }

    PointList2(const Eigen::Matrix<T, D, Eigen::Dynamic> &x): Base(x.cols()) {
        for( uint idx = 0 ; idx<x.cols() ; idx++ )
            (*this)[idx] = x.col(idx) ;
    }

    PointList2(const cv::Mat &src) {
         assert(src.cols == D);
         this->resize(src.rows) ;
         cv::Mat dst(src.rows, src.cols, cv::DataType<T>::type, &(*this)[0], D*(size_t)(sizeof(T)));
         src.convertTo(dst, dst.type());
         assert( dst.data == (uchar*)this->data());
    }

    Point center() const {
        return Point() ;
    }

    void axes(double &l1, Point &v1, double &l2, Point &v2) const ;

    // Procrustes analysis
    // Find the transform  that aligns this shape with the other one:
    // this' = T(s) * T(theta) * this + T(tx, ty)
    Eigen::Affine2d align(const PointList2<T, D> &other) const ;
/*
    void transform(const Eigen::Affine2d &xf) {
        mat_ = xf.cast<float>() * mat_.transpose() ;
    }
*/
    std::pair< Point, Point > bbox() const {
        //return std::make_pair(mat_.colwise().minCoeff(), mat_.colwise().maxCoeff()) ;
    }

    double norm() const {  ; }
  //  void translate(const Point<T, D> &offset) ;
    void scale(double s) ;

    cv::Mat toCVMat() const {
    //     return cv::Mat(mat_.rows(), 1, cv::DataType< cv::Vec<T, D> >::type, (void*)mat_.data(), mat_.stride() * sizeof(T));
    }

protected:

//    container_t<T, D> data_ ;
} ;

using pointlist3f = PointList2<float, 3> ;
using pointlist4f = PointList2<float, 4, Eigen::aligned_allocator<Eigen::Vector4f>> ;


int main(int argc, char *argv[]) {
    pointlist3f a {{ 1.0, 2, 3}, {4.0, 5.0, 6}};
    float data[] = {1.0, 2.0, 3.0, 4.0, 5, 6, 7, 8, 9, 10, 11, 12} ;
    pointlist4f b { data, 2, false } ;


    Eigen::MatrixX3f m(4, 3) ;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9,
         10, 11, 12;

    Eigen::Matrix3Xf m2(3, 4) ;
    m2 << 1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12;


    cout << m << endl ;
    pointlist3f mm(m2) ;

    cv::Mat f(4, 3, CV_32FC1, data) ;

    cout << f << endl ;

    pointlist3f g(f) ;

    Eigen::Vector3f q(1, 2, 3) ;
   cout << "ok" <<endl ;
}

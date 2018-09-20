#include <cvx/util/geometry/util.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace util {

Matrix4f lookAt(const Vector3f & eye,  const Vector3f &center, const Vector3f &up)
{
    Vector3f f = (center - eye).normalized();
    Vector3f s = f.cross(up).normalized();
    Vector3f u = s.cross(f) ;

    Matrix4f res ;
    res << s.x(), s.y(), s.z(), -s.dot(eye),
            u.x(), u.y(), u.z(), -u.dot(eye),
            -f.x(), -f.y(), -f.z(), f.dot(eye),
            0, 0, 0, 1 ;

    return res ;
}


Eigen::Hyperplane<float, 3> fitPlaneToPoints(const vector<Vector3f> &pts) {
    uint n_pts = pts.size() ;
    assert(n_pts >= 3) ;

    if ( n_pts == 3 )
        return Eigen::Hyperplane<float, 3>::Through(pts[0], pts[1], pts[2]) ;
    else {
        Eigen::Map<Matrix<float,Dynamic,3,RowMajor> > mat((float *)pts.data(), pts.size(), 3);
        VectorXf centroid = mat.colwise().mean();
        MatrixXf centered = mat.rowwise() - centroid.adjoint() ;
        MatrixXf cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
        JacobiSVD<Matrix3f> svd(cov, ComputeFullU);
        Vector3f normal = svd.matrixU().col(2);
        return Eigen::Hyperplane<float, 3>(normal, centroid) ;
    }
}


}}

#include <cvx/util/pcl/align.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace util {

using Eigen::Vector3f ;

Isometry3f alignRigid(const Matrix3Xf &P, const Matrix3Xf &Q) {

    // Default output
    Isometry3f A;
    A.linear() = Matrix3f::Identity(3, 3);
    A.translation() = Vector3f::Zero();

    if (P.cols() != Q.cols())
        throw "Find3DAffineTransform(): input data mis-match";

    // Center the data
    Vector3f p = P.rowwise().mean();
    Vector3f q = Q.rowwise().mean();

    Matrix3Xf X = P.colwise() - p;
    Matrix3Xf Y = Q.colwise() - q;

    // SVD
    MatrixXf Cov = X*Y.transpose();
    JacobiSVD<MatrixXf> svd(Cov, ComputeThinU | ComputeThinV);

    // Find the rotation, and prevent reflections
    Matrix3f I = Matrix3f::Identity(3, 3);
    double d = (svd.matrixV()*svd.matrixU().transpose()).determinant();
    (d > 0.0) ? d = 1.0 : d = -1.0;
    I(2, 2) = d;

    Matrix3f R = svd.matrixV()*I*svd.matrixU().transpose();

    // The final transform
    A.linear() = R;
    A.translation() = q - R*p;

    return A;
}

Isometry3f alignRigid(const vector<Vector3f> &src, const vector<Vector3f> &dst) {

    assert( src.size() == dst.size() ) ;

    Eigen::Map<Matrix3Xf> m_src((float *)src.data(), 3, src.size())  ;
    Eigen::Map<Matrix3Xf> m_dst((float *)dst.data(), 3, dst.size())  ;

    return alignRigid(m_src, m_dst) ;
}



}}

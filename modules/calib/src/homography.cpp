#include <cvx/calib/homography.hpp>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <cvx/util/math/solvers/lm.hpp>

#include <iostream>

using namespace cvx::util ;
using namespace Eigen ;
using namespace std ;

namespace cvx { namespace camera {


static void normalize_points(const std::vector<cv::Point2f> &pts, std::vector<cv::Point2f> &npts, Matrix3d &T) {

    size_t N = pts.size() ;
    cv::Point2f center = std::accumulate(pts.begin(), pts.end(),cv::Point2f {0, 0}) ;
    center /= (float)N ;

    // scale pts so that their norm is sqrt(2) ;

    float nrm = 0 ;
    std::for_each (pts.begin(), pts.end(), [&](const cv::Point2f &v) {
        nrm += (v - center).dot(v - center) ;
    });

    nrm /= (float)N ;

    double scale = sqrt(2.0)/sqrt(nrm) ;

    T << scale, 0, -scale * center.x,
            0, scale, -scale * center.y,
            0, 0,  1 ;

    npts.resize(N) ;

    std::transform(pts.begin(), pts.end(), npts.begin(), [&] (const cv::Point2f &p ){ return ( p - center )*scale ; }) ;

}



void HomographyEstimator::solve(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, Eigen::Matrix3d &H) {
    assert( pts1.size() == pts2.size() ) ;

    const size_t N = pts1.size();

    std::vector<cv::Point2f> npts1, npts2 ;
    Matrix3d T1, T2;

    // center points to origin and rescale
    normalize_points(pts1, npts1, T1) ;
    normalize_points(pts2, npts2, T2) ;

    // Setup constraint matrix.
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*N, 9);

    for (size_t i = 0; i < N; ++i) {
        const double u1 = npts1[i].x ;
        const double v1 = npts1[i].y ;
        const double u2 = npts2[i].x ;
        const double v2 = npts2[i].y ;

        size_t j = 2*i ;
        A(j, 0) = u1 ;
        A(j, 1) = v1 ;
        A(j, 2) = 1 ;
        A(j, 6) = -u2 * u1;
        A(j, 7) = -u2 * v1;
        A(j, 8) = -u2 ;

        ++j ;

        A(j, 3) = -u1;
        A(j, 4) = -v1;
        A(j, 5) = -1;
        A(j, 6) = v2 * u1;
        A(j, 7) = v2 * v1;
        A(j, 8) = v2;
    }

    // Solve for the nullspace of the constraint matrix.
    Eigen::JacobiSVD< Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

    const Eigen::VectorXd nullspace = svd.matrixV().col(8);
    Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data());

    H = (T2.inverse() * H_t.transpose() * T1).eval();
    H /= H(2, 2) ;
}

void HomographyEstimator::residuals(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, const Matrix3d &H, std::vector<double> &res)
{
    assert(pts1.size() == pts2.size()) ;

    size_t N = pts1.size() ;
    Matrix3d Hi = H.inverse() ;

    double resid = 0.0 ;
    for( size_t i=0 ; i<N ; i++ ) {
        Eigen::Vector2d v1(pts1[i].x, pts1[i].y) ;
        Eigen::Vector2d v2(pts2[i].x, pts2[i].y) ;

        res.push_back(( (H * v1.homogeneous()).hnormalized() - v2).norm() +
                      (( Hi * v2.homogeneous()).hnormalized() - v1).norm() ) ;
    }
}

class RansacHomographyEstimator {
public:
    typedef Eigen::Matrix3d Params ;
    RansacHomographyEstimator(HomographyEstimator &est, const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2): pts1_(pts1), pts2_(pts2), estimator_(est) {}

    static const size_t minSamples = 4 ;

    bool fit(const std::vector<size_t> &subset, Params &H) {
        // we should chech hear if the points are collinear
        std::vector<cv::Point2f> pts1, pts2 ;
        for (size_t i: subset ) {
            pts1.emplace_back(pts1_[i]) ;
            pts2.emplace_back(pts2_[i]) ;
        }

        estimator_.solve(pts1, pts2, H) ;
        return true ;
    }

    double findInliers(const Params &params, double threshold, std::vector<size_t> &inliers) {
        std::vector<double> res;
        estimator_.residuals(pts1_, pts2_, params, res) ;
        double total = 0 ;
        for( uint i=0 ; i<res.size() ; i++ ) {
            if ( res[i] < threshold ) {
                inliers.push_back(i) ;
                total += res[i] ;
            }
        }

        total /= (double)inliers.size() ;
        return total ;
    }

    const std::vector<cv::Point2f> &pts1_ ;
    const std::vector<cv::Point2f> &pts2_ ;
    HomographyEstimator &estimator_ ;
};

bool HomographyEstimator::solveRansac(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, Eigen::Matrix3d &H, std::vector<size_t> &inliers) {
    RANSAC solver ;
    RansacHomographyEstimator estimator(*this, pts1, pts2) ;

    return solver.estimate(pts1.size(), estimator, inliers, H) ;
}


class LMHomographyEstimator {

public:
    LMHomographyEstimator(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, const std::vector<size_t> &inliers): pts1_(pts1), pts2_(pts2), inliers_(inliers) {}

    size_t terms() const { return 2 * inliers_.size() ; }

    void errors(const VectorXd &h, VectorXd &e) {
        size_t N = inliers_.size() ;

        for( size_t i=0 ; i<N ; i++ ) {
            size_t idx = inliers_[i] ;
            float x1 = pts1_[idx].x ;
            float y1 = pts1_[idx].y ;
            float x2 = pts2_[idx].x ;
            float y2 = pts2_[idx].y ;

            double w = h[6]*x1 + h[7]*y1 + 1.0 ;
            w = ( w > std::numeric_limits<double>::epsilon() ) ? 1.0/w : 0.0 ;

            size_t j = 2*i ;
            double xi = (h[0]*x1 + h[1]*y1 + h[2])*w;
            double yi = (h[3]*x1 + h[4]*y1 + h[5])*w;

            e[j] = xi - x2 ;
            e[j+1] = yi - y2 ;
        }
    }

    void jacobian(const VectorXd &h, MatrixXd &J) {

        size_t N = inliers_.size() ;

        for( size_t i=0 ; i<N ; i++ ) {
            size_t idx = inliers_[i] ;
            float x1 = pts1_[idx].x ;
            float y1 = pts1_[idx].y ;

            double w = h[6]*x1 + h[7]*y1 + 1.0 ;
            w = ( w > std::numeric_limits<double>::epsilon() ) ? 1.0/w : 0.0 ;

            size_t j = 2*i ;
            double xi = (h[0]*x1 + h[1]*y1 + h[2])*w;
            double yi = (h[3]*x1 + h[4]*y1 + h[5])*w;

            J.row(j) << x1*w,  y1*w, w, 0, 0, 0, -x1*xi*w, -y1*xi*w ;
            J.row(j+1) << 0, 0, 0, x1*w,  y1*w, w, -x1*yi*w, -y1*yi*w ;
        }
    }

private:

    const std::vector<cv::Point2f> &pts1_ ;
    const std::vector<cv::Point2f> &pts2_ ;
    const std::vector<size_t> &inliers_ ;

};


bool HomographyEstimator::solveLM(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, const std::vector<size_t> &inliers, Eigen::Matrix3d &H) {

    LMHomographyEstimator estimator(pts1, pts2, inliers) ;

    typedef LMSolver<double, LMHomographyEstimator> Solver ;
    Solver::Parameters params ;
    params.f_tol_ = params_.lm_params_.ftol_ ;
    params.max_iter_ = params_.lm_params_.max_iter_ ;
    Solver lm(params) ;

    VectorXd h(8) ;
    h << H(0, 0), H(0, 1), H(0, 2), H(1, 0), H(1, 1), H(1, 2), H(2, 0), H(2, 1) ;

    lm.minimizeDer(estimator, h);

    H << h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0 ;

    return true ;
}



}}

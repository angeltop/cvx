#include <cvx/calib/calibration.hpp>
#include <cvx/calib/pattern.hpp>
#include <cvx/calib/pose.hpp>
#include <cvx/util/misc/path.hpp>

#include <opencv2/opencv.hpp>
#include <iterator>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;

using namespace std ;
using namespace cvx::util ;

namespace cvx { namespace camera {

void CameraCalibration::Data::save(const string &out_path) const {
    ofstream strm(out_path) ;

    strm << fsize_.width << ' ' << fsize_.height << endl ;

    // write 3D coordinates of calibration points

    strm << coords_.size() << endl ;
    for( const cv::Point3f &p: coords_)
        strm << p.x << ' ' << p.y << ' ' << p.z << endl ;

    // write to file

    strm << markers_.size() << endl ;

    uint k = 0 ;
    for( const auto &v: markers_ ) {
        strm << image_paths_[k++] << endl ;
        strm << v.size() << endl ;
        for( const CalibrationPattern::Marker &m: v ) {
            strm << m.pt_.x << ' ' <<  m.pt_.y << ' ' << m.id_ << endl ;
        }
    }
}

void CameraCalibration::Data::load(const string &calib_file) {
    ifstream strm(calib_file) ;

    strm >> fsize_.width >> fsize_.height ;

    size_t n_coords, n_views ;
    strm >> n_coords ;
    coords_.resize(n_coords) ;
    for( size_t i = 0 ; i<n_coords ; i++) {
        float x, y, z ;
        strm >> x >> y >> z ;
        coords_[i] = cv::Point3f(x, y, z) ;
    }
    strm >> n_views ;

    image_paths_.resize(n_views) ;
    markers_.resize(n_views) ;

    for( size_t i = 0 ; i<n_views ; i++) {
        strm >> image_paths_[i] ;
        size_t n_markers ;
        strm >> n_markers ;
        markers_[i].resize(n_markers) ;
        for( size_t j = 0 ; j<n_markers ; j++ ) {
            float x, y ;
            int idx ;
            strm >> x >> y >> idx ;
            markers_[i][j].pt_ = cv::Point2f(x, y) ;
            markers_[i][j].id_ = idx ;
        }
    }

}


void CameraCalibration::detect(const vector<string> &images, const CalibrationPattern &pattern, Data &data)
{
    // run detector and collect markers

    data.coords_ = pattern.coords() ;

    vector<vector<CalibrationPattern::Marker>> pts ;

    for( const string &ipath: images ) {

        cv::Mat im = cv::imread(ipath) ;
        if ( !im.data  ) continue ;

        data.fsize_ = im.size() ; // assume all images have the same size

        vector<CalibrationPattern::Marker> ipts = pattern.findPoints(im) ;

        if ( ipts.empty() ) continue ;

        cout << ipath << endl ;
        pattern.draw(im, ipts) ;
        cv::imwrite("/tmp/" + Path(ipath).name(), im) ;

        data.markers_.push_back(std::move(ipts)) ;
        data.image_paths_.push_back(ipath) ;
    }
}

extern double computeReprojectionError(
        const vector<cv::Point3f>& objectPoints,
        const vector<cv::Point2f>& imagePoints,
        const cv::Mat rvec, const cv::Mat & tvec,
        const PinholeCamera &cam ) ;

static double reprojectionError(const vector<cv::Point3f>& objectPoints,
                                const vector<cv::Point2f>& imagePoints,
                                const Eigen::Matrix4d t,
                                const PinholeCamera &cam
                                ) {
    double total = 0 ;
    for( uint i=0 ; i<objectPoints.size() ; i++ ) {
        const cv::Point3f &p = objectPoints[i] ;
        const cv::Point2f &q = imagePoints[i] ;
        Eigen::Vector4d pj = t * Eigen::Vector4d(p.x, p.y, p.z, 1.0) ;
        cv::Point3d pp{pj.x(), pj.y(), pj.z()} ;

        cv::Mat_<double> dist(cam.getDistortion()) ;

        double k1 = dist(0, 0) ;
        double k2 = dist(1, 0) ;
        double p1 = dist(2, 0) ;
        double p2 = dist(3, 0) ;
        double k3 = dist(4, 0) ;

        double focal_length_x = cam.fx() ;
        double focal_length_y = cam.fy() ;
        double principal_point_x = cam.cx() ;
        double principal_point_y = cam.cy() ;

        double x = pp.x/pp.z;
        double y = pp.y/pp.z;

        // Apply distortion to the normalized points to get (xd, yd).
        double r2 = x*x + y*y;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double r_coeff = 1 + k1*r2 + k2*r4 + k3*r6;
        double xd = x * r_coeff + double(2)*p1*x*y + p2*(r2 + double(2)*x*x);
        double yd = y * r_coeff + double(2)*p2*x*y + p1*(r2 + double(2)*y*y);
        // Apply focal length and principal point to get the final image coordinates.
        double image_x = focal_length_x * xd + principal_point_x;
        double image_y = focal_length_y * yd + principal_point_y;

        double error =  (image_x - q.x)*(image_x - q.x) + (image_y - q.y)*(image_y - q.y)  ;
        total += error ;

    }

    return sqrt(total/objectPoints.size()) ;
}

double CameraCalibration::run(const Data &calib_data, PinholeCamera &cam, vector<Eigen::Matrix4d> &extrinsics, int flags)
{
    cv::Mat cameraMatrix = cam.getMatrix();
    cv::Mat distCoeffs = cam.getDistortion().clone() ;

    cameraMatrix.at<double>(0, 0) = cam.fx() ;
    cameraMatrix.at<double>(1, 1) = cam.fy();
    cameraMatrix.at<double>(0, 2) = calib_data.fsize_.width/2.0 ;
    cameraMatrix.at<double>(1, 2) = calib_data.fsize_.height/2.0 ;

    vector<vector<cv::Point3f> > object_pts ;
    vector<vector<cv::Point2f> > image_pts ;

    size_t n_views = calib_data.markers_.size() ;

    object_pts.resize(n_views) ;
    image_pts.resize(n_views) ;

    for( size_t i = 0 ; i < n_views ; i++ ) {
        uint n_markers = calib_data.markers_[i].size() ;
        object_pts[i].resize(n_markers) ;
        image_pts[i].resize(n_markers) ;

        for( size_t j=0 ; j < n_markers ; j++ ) {
            const auto &m = calib_data.markers_[i][j] ;
            int idx = m.id_ ;
            object_pts[i][j] = calib_data.coords_[idx] ;
            image_pts[i][j] = m.pt_ ;
        }
    }

    vector<cv::Mat> rvec, tvec ;

    double rms = cv::calibrateCamera(object_pts, image_pts, calib_data.fsize_, cameraMatrix, distCoeffs, rvec, tvec,
                                     flags,
                                     cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)) ;

    cout << rms << endl ;
    cam.setSize(calib_data.fsize_) ;
    cam.setMatrix(cameraMatrix) ;
    cam.setDistortion(distCoeffs) ;

    double total = 0.0 ;
    for( uint i=0 ; i<n_views ; i++ ) {
        double e = computeReprojectionError(object_pts[i], image_pts[i], rvec[i], tvec[i], cam) ;
        extrinsics.emplace_back(rodriguesToAffine(rvec[i], tvec[i])) ;

        double re = reprojectionError(object_pts[i], image_pts[i], extrinsics[i], cam) ;
        total += e ;
    }

    return total / n_views ;

}

enum {
    OFFSET_FOCAL_LENGTH_X = 0,
    OFFSET_FOCAL_LENGTH_Y = 1,
    OFFSET_PRINCIPAL_POINT_X = 2,
    OFFSET_PRINCIPAL_POINT_Y = 3,
    OFFSET_K1 = 4,
    OFFSET_K2 = 5,
    OFFSET_K3 = 6,
    OFFSET_P1 = 7,
    OFFSET_P2= 8,
};

template <typename T>
inline void ApplyRadialDistortionCameraIntrinsics(const T &focal_length_x,
                                                  const T &focal_length_y,
                                                  const T &principal_point_x,
                                                  const T &principal_point_y,
                                                  const T &k1,
                                                  const T &k2,
                                                  const T &k3,
                                                  const T &p1,
                                                  const T &p2,
                                                  const T &normalized_x,
                                                  const T &normalized_y,
                                                  T *image_x,
                                                  T *image_y) {
    T x = normalized_x;
    T y = normalized_y;
    // Apply distortion to the normalized points to get (xd, yd).
    T r2 = x*x + y*y;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
    T xd = x * r_coeff + T(2)*p1*x*y + p2*(r2 + T(2)*x*x);
    T yd = y * r_coeff + T(2)*p2*x*y + p1*(r2 + T(2)*y*y);
    // Apply focal length and principal point to get the final image coordinates.
    *image_x = focal_length_x * xd + principal_point_x;
    *image_y = focal_length_y * yd + principal_point_y;
}

struct OpenCVReprojectionError {
    OpenCVReprojectionError(const double observed_x, const double observed_y)
        :observed_x(observed_x), observed_y(observed_y) {

    }

    template <typename T>
    bool operator()(const T* const intrinsics,
                    const T* const R_t,  // Rotation denoted by angle axis
                    // followed with translation
                    const T *const X,
                    T* residuals) const {

        // Unpack the intrinsics.
        const T& focal_length_x      = intrinsics[OFFSET_FOCAL_LENGTH_X];
        const T& focal_length_y      = intrinsics[OFFSET_FOCAL_LENGTH_Y];
        const T& principal_point_x = intrinsics[OFFSET_PRINCIPAL_POINT_X];
        const T& principal_point_y = intrinsics[OFFSET_PRINCIPAL_POINT_Y];
        const T& k1                = intrinsics[OFFSET_K1];
        const T& k2                = intrinsics[OFFSET_K2];
        const T& k3                = intrinsics[OFFSET_K3];
        const T& p1                = intrinsics[OFFSET_P1];
        const T& p2                = intrinsics[OFFSET_P2];

        // Compute projective coordinates: x = RX + t.
        T x[3];

        ceres::AngleAxisRotatePoint(R_t, X, x);
        x[0] += R_t[3];
        x[1] += R_t[4];
        x[2] += R_t[5];

        // Compute normalized coordinates: x /= x[2].
        T xn = x[0] / x[2];
        T yn = x[1] / x[2];
        T predicted_x, predicted_y;
        // Apply distortion to the normalized points to get (xd, yd).
        // TODO(keir): Do early bailouts for zero distortion; these are expensive
        // jet operations.
        ApplyRadialDistortionCameraIntrinsics(focal_length_x,
                                              focal_length_y,
                                              principal_point_x,
                                              principal_point_y,
                                              k1, k2, k3,
                                              p1, p2,
                                              xn, yn,
                                              &predicted_x,
                                              &predicted_y);
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }
    const double observed_x;
    const double observed_y;

};


static vector<Vec6> packCamerasRotationAndTranslation(const vector<Eigen::Matrix4d> &extrinsics) {

    vector<Vec6> all_cameras_R_t;

    all_cameras_R_t.resize(extrinsics.size()) ;
    for( size_t i=0 ; i<extrinsics.size() ; i++ ) {
        const Eigen::Matrix4d &mat = extrinsics[i] ;
        Eigen::Matrix3d R = mat.block<3, 3>(0, 0) ;
        Eigen::Vector3d T = mat.block<3, 1>(0, 3) ;

        ceres::RotationMatrixToAngleAxis(&R(0, 0), &all_cameras_R_t[i](0));
                all_cameras_R_t[i].tail<3>() = T;
    }
    return all_cameras_R_t;
}

// Convert cameras rotations fro mangle axis back to rotation matrix.
void unpackCamerasRotationAndTranslation(const vector<Vec6> &all_cameras_R_t,
                                         vector<Eigen::Matrix4d> &extrinsics) {

    for( size_t i=0 ; i<extrinsics.size() ; i++ ) {
        Eigen::Matrix4d &mat = extrinsics[i] ;

        Eigen::Matrix3d R ;
        Eigen::Vector3d T ;

        ceres::AngleAxisToRotationMatrix(&all_cameras_R_t[i](0), &R(0, 0)) ;
                T = all_cameras_R_t[i].tail<3>();

        mat.block<3, 3>(0, 0) = R ;
        mat.block<3, 1>(0, 3) = T ;
    }
}

// refine the camera calibration by means of bundle adjustment over camera intrinsics, extrinsics and 3D pattern coordinates

double CameraCalibration::refine(const Data &calib_data, PinholeCamera &cam, vector<Eigen::Matrix4d> &extrinsics, bool intrinsics) {

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    vector<Vec6> all_cameras_R_t = packCamerasRotationAndTranslation(extrinsics);

    size_t n_views = calib_data.markers_.size() ;

    double camera_intrinsics[19];

    camera_intrinsics[OFFSET_FOCAL_LENGTH_X] = cam.fx();
    camera_intrinsics[OFFSET_FOCAL_LENGTH_Y] = cam.fy();
    camera_intrinsics[OFFSET_PRINCIPAL_POINT_X] = cam.cx() ;
    camera_intrinsics[OFFSET_PRINCIPAL_POINT_Y] = cam.cy() ;

    cv::Mat_<double> dist(cam.getDistortion()) ;
    camera_intrinsics[OFFSET_K1] = dist(0, 0) ;
    camera_intrinsics[OFFSET_K2] = dist(0, 1) ;
    camera_intrinsics[OFFSET_P1] = dist(0, 2) ;
    camera_intrinsics[OFFSET_P2] = dist(0, 3) ;
    camera_intrinsics[OFFSET_K3] = dist(0, 4) ;

    vector<Vec3> coords ;

    for( const auto &p: calib_data.coords_ ) {
        coords.emplace_back(p.x, p.y, p.z) ;
    }

    vector<ceres::ResidualBlockId> block_ids ;

    for( size_t i=0 ; i<n_views ; i++ ) {
        size_t n_markers = calib_data.markers_[i].size() ;

        for( size_t j=0 ; j<n_markers ; j++ ) {
            const CalibrationPattern::Marker &marker = calib_data.markers_[i][j] ;

            ceres::ResidualBlockId block_id = problem.AddResidualBlock(new ceres::AutoDiffCostFunction<
                                                                       OpenCVReprojectionError, 2, 9, 6, 3>(
                                                                           new OpenCVReprojectionError(marker.pt_.x, marker.pt_.y)),
                                                                       NULL,
                                                                       camera_intrinsics,
                                                                       &all_cameras_R_t[i](0),
                                                                       &coords[marker.id_](0)
                                                                       );

            block_ids.push_back(block_id) ;
        }
    }


  //        problem.SetParameterBlockConstant(&all_cameras_R_t[0](0));

    if ( !intrinsics )
        problem.SetParameterBlockConstant(camera_intrinsics);
    else {
       std::vector<int> constant_intrinsics;

        constant_intrinsics.push_back(OFFSET_K3);
        ceres::SubsetParameterization *subset_parameterization =
                new ceres::SubsetParameterization(9, constant_intrinsics);
        problem.SetParameterization(camera_intrinsics, subset_parameterization);

    }

    // Configure the solver.
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100;
    options.function_tolerance = 1.0e-10 ;
    options.gradient_tolerance = 1.0e-10 ;
    options.minimizer_progress_to_stdout = true;

    // Solve!

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // Copy rotations and translations back.

    unpackCamerasRotationAndTranslation(all_cameras_R_t, extrinsics) ;

    ceres::Problem::EvaluateOptions eoptions;
    eoptions.residual_blocks = block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(eoptions, &total_cost, &residuals, nullptr, nullptr);

    if ( intrinsics ) {
        cv::Mat_<double> P = cv::Mat_<double>::zeros(3, 3) ;
        P(0, 0) = camera_intrinsics[OFFSET_FOCAL_LENGTH_X] ;
        P(1, 1) = camera_intrinsics[OFFSET_FOCAL_LENGTH_Y] ;
        P(2, 2) = 1.0 ;
        P(0, 2) = camera_intrinsics[OFFSET_PRINCIPAL_POINT_X] ;
        P(1, 2) = camera_intrinsics[OFFSET_PRINCIPAL_POINT_Y] ;

        dist(0, 0) = camera_intrinsics[OFFSET_K1] ;
        dist(0, 1) = camera_intrinsics[OFFSET_K2] ;
        dist(0, 2) = camera_intrinsics[OFFSET_P1] ;
        dist(0, 3) = camera_intrinsics[OFFSET_P2] ;
        dist(0, 4) = camera_intrinsics[OFFSET_K3] ;

        cam.setDistortion(dist) ;
        cam.setMatrix(P) ;

    }

    return sqrt(total_cost/residuals.size()) ;
}

}}

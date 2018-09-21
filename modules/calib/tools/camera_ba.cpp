#include <iostream>

#include <cvx/util/misc/path.hpp>
#include <cvx/calib/calibration.hpp>
#include <cvx/calib/pose.hpp>
#include <cvx/util/misc/arg_parser.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <regex>

using namespace std ;
using namespace cvx::camera ;
using namespace cvx::util ;
using namespace Eigen ;

template <typename T>
inline void ApplyRadialDistortionCameraIntrinsics(const double &focal_length_x,
                                                  const double &focal_length_y,
                                                  const double &principal_point_x,
                                                  const double &principal_point_y,
                                                  const double &k1,
                                                  const double &k2,
                                                  const double &k3,
                                                  const double &p1,
                                                  const double &p2,
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

    const PinholeCamera &cam_ ;
    cv::Point2d img_ ;
    cv::Point3d obj_ ;
    Matrix4d trans_ ;

    OpenCVReprojectionError(const PinholeCamera &cam, const cv::Point2d &img, const Affine3d &trans)
        :img_(img), cam_(cam), trans_(trans.matrix()) {

    }

    template <typename T>
    bool operator()(const T* const q,  // Rotation quaternion
                    const T* const t, // translation
                    const T *const X,
                    T* residuals) const {

        // Unpack the intrinsics.
        const double focal_length_x      = cam_.fx();
        const double focal_length_y      = cam_.fy();
        const double principal_point_x = cam_.cx() ;
        const double principal_point_y = cam_.cy() ;

        cv::Mat dist = cam_.getDistortion() ;
        const double k1 = dist.at<double>(0, 0) ;
        const double k2 = dist.at<double>(0, 1) ;
        const double p1 = dist.at<double>(0, 2) ;
        const double p2 = dist.at<double>(0, 3) ;
        const double k3 = dist.at<double>(0, 4) ;

        // Compute projective coordinates: x = RX + t.

        T x[3], Xt[3] ;
        Matrix<T, 4, 1> R ;
        R[0] = X[0] ;
        R[1] = X[1] ;
        R[2] = X[2] ;
        R[3] = (T)1.0 ;

        Matrix<T, 4, 1> C = trans_.cast<T>() * R ;

        Xt[0] = C[0] ;
        Xt[1] = C[1] ;
        Xt[2] = C[2] ;

        ceres::QuaternionRotatePoint(q, Xt, x);
        x[0] += t[0];
        x[1] += t[1];
        x[2] += t[2];

#if 0
        T x[3], X[3];

        Matrix<T, 4, 1> R ;
        R[0] = obj_.x + offset[0] ;
        R[1] = obj_.y + offset[1] ;
        R[2] = obj_.z + offset[2] ;
        R[3] = (T)1.0 ;

        Matrix<T, 4, 1> C = trans_.cast<T>() * R ;

        X[0] = C[0] ;
        X[1] = C[1] ;
        X[2] = C[2] ;

        ceres::QuaternionRotatePoint(q, X, x);

        x[0] += t[0];
        x[1] += t[1];
        x[2] += t[2];
#endif
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
        residuals[0] = predicted_x - T(img_.x);
        residuals[1] = predicted_y - T(img_.y);
        return true;
    }
};


void bundleAdjustment(const PinholeCamera &cam, const CameraCalibration::Data &data, vector<Affine3d> gripper_to_base, Affine3d &pose, Vector3d &off) {

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    vector<ceres::ResidualBlockId> block_ids ;

    uint n_views = data.markers_.size() ;

    Quaterniond q(pose.rotation()) ;
    Vector3d t = pose.translation() ;

    double p[7] = { q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z()} ;
    double offset[3] = { 0, 0, 0 } ;
    std::vector<double> coords ;
    for( size_t i=0 ; i<data.coords_.size() ; i++ ) {
        const cv::Point3f &p = data.coords_[i] ;
        coords.push_back(p.x) ;
        coords.push_back(p.y) ;
        coords.push_back(p.z) ;
    }


    for( size_t i=0 ; i<n_views ; i++ ) {
        size_t n_markers = data.markers_[i].size() ;

        for( size_t j=0 ; j<n_markers ; j++ ) {
            const CalibrationPattern::Marker &marker = data.markers_[i][j] ;

            int idx = marker.id_ ;
            cv::Point3f X = data.coords_[idx] ;
            cv::Point2f x = marker.pt_ ;

            ceres::ResidualBlockId block_id = problem.AddResidualBlock(new ceres::AutoDiffCostFunction<
                                                                       OpenCVReprojectionError, 2, 4, 3, 3>(
                                                                           new OpenCVReprojectionError(cam, x, gripper_to_base[i])),
                                                                       NULL,
                                                                       p,
                                                                       p + 4,
                                                                       &coords[3*idx]
                                                                       );

            block_ids.push_back(block_id) ;
        }
    }


    // ceres deletes the object allocated here for the user
    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(p, quaternionParameterization);

    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = false;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100;
    options.function_tolerance = 1.0e-20 ;
    options.gradient_tolerance = 1.0e-20 ;
    options.parameter_tolerance = 1.0e-20 ;
    options.minimizer_progress_to_stdout = true;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    ceres::Problem::EvaluateOptions eoptions;
    eoptions.residual_blocks = block_ids;
    double total_cost = 0.0;
    vector<double> residuals;
    problem.Evaluate(eoptions, &total_cost, &residuals, nullptr, nullptr);


    Eigen::Quaterniond qx(p[0], p[1], p[2], p[3]);
    Eigen::Vector3d tr;
    tr << p[4], p[5], p[6];

    pose = Affine3d(Translation3d(tr) * qx.toRotationMatrix()) ;
  //  pose = pose.inverse() ;
    cout << pose.matrix() << endl ;
    off.x() = offset[0] ;
    off.y() = offset[1] ;
    off.z() = offset[2] ;

    cout << off << endl ;

}

void loadPoses(const PinholeCamera &cam, const CameraCalibration::Data &cdata,  vector<Affine3d> &target_to_sensor,
               vector<Affine3d> &gripper_to_base )
{
    const string img_file_prefix = "cap" ;
    const string img_file_suffix = "png" ;
    const string pose_file_prefix = "pose" ;
    const string pose_file_suffix = "txt" ;

    std::string file_name_regex =  img_file_prefix + "([[:digit:]]+[a]?)\\." + img_file_suffix ;

    std::regex rx(file_name_regex) ;

    for ( uint k=0 ; k<cdata.image_paths_.size() ; k++ ) {

       uint n_markers = cdata.markers_[k].size() ;

    //   if ( n_markers < 40 ) continue ;

       vector<cv::Point3f> object_pts ;
       vector<cv::Point2f> image_pts ;
       object_pts.resize(n_markers) ;
       image_pts.resize(n_markers) ;


        for( size_t j=0 ; j < n_markers ; j++ ) {
            const auto &m = cdata.markers_[k][j] ;
            int idx = m.id_ ;
            object_pts[j] = cdata.coords_[idx] ;
            image_pts[j] = m.pt_ ;
        }

        double err ;
       Eigen::Affine3d pose = estimatePosePlanar(image_pts, object_pts, cam, err) ;

        Path ipath(cdata.image_paths_[k]) ;

        target_to_sensor.push_back(pose) ;

        string folder = ipath.parent() ;
        string iname = ipath.name() ;

        std::smatch sm ;

        // test of this is a valid filename

        if ( !std::regex_match(iname, sm, rx ) ) continue ;

        string sp = sm[1] ;

        std::string pose_file_name = pose_file_prefix + sp + "." + pose_file_suffix ;

        cout << pose_file_name << endl ;

        ifstream strm(folder + '/' + pose_file_name) ;

        Matrix4d tr ;

        for (int row = 0; row < 4; row++)
               for (int col = 0; col < 4; col++) {
                   strm >> tr(row, col) ;
               }

        gripper_to_base.push_back(Affine3d(tr.inverse())) ;
     //   gripper_to_base.push_back(Affine3d(tr)) ;
    }
}

void drawResiduals(const PinholeCamera &cam, const CameraCalibration::Data &cdata, const vector<Affine3d> &gripper_to_base, const Affine3d &sensor_to_base,
                   const std::vector<cv::Point3f> &coords) {

    for( uint i=0 ; i<cdata.image_paths_.size() ; i++ )  {

         cout << cdata.image_paths_[i] << "--> " ;

        for( uint j=0 ; j<4 ; j++ ) {
          Vector3d m(coords[j].x, coords[j].y, coords[j].z) ;

            Vector3d p = sensor_to_base.inverse() * gripper_to_base[i] * m ;

            cv::Point2d pj = cam.project(cv::Point3d(p.x(), p.y(), p.z())) ;

            cout << "[" << j << "]" << pj << ' ' ;
        }

        cout << endl ;

    }

}

istream &operator >> (istream &strm, cv::Size &sz) {
    strm >> sz.width >> sz.height ;
    return strm ;
}

int main(int argc, char *argv[]) {

    ArgumentParser args ;

    string data_folder, camera_intrinsics, output_file ;
    bool compute_markers = false ;
    cv::Size pattern_grid(1, 1) ;
    float pattern_width = 0.062, pattern_gap = 0.01 ;
    bool print_help = false ;

    args.setDescription("Usage: camera_ba [options]") ;
    args.addOption("-h|--help", print_help).setMaxArgs(0).setDescription("print this help message").setImplicit("true") ;
    args.addOption("--data", data_folder).required().setName("<folder>").setDescription("Folder with captured images and robot tip poses") ;
    args.addOption("--markers", compute_markers).setMaxArgs(0).setDescription("Recompute markers positions on images").setImplicit("true") ;
    args.addOption("--grid", pattern_grid).setMinArgs(2).setMaxArgs(2).setName("<gx> <gy>").setDescription("Number of tags in x and y dimensions") ;
    args.addOption("--width", pattern_width).setName("<arg>").setDescription("Width of the AprilTag square in meters") ;
    args.addOption("--gap", pattern_gap).setName("<arg>").setDescription("Gap between AprilTags in the grid in meters") ;
    args.addOption("--camera", camera_intrinsics).required().setName("<filename>").setDescription("Path to camera intrinsics file") ;
    args.addOption("--out", output_file).required().setName("<output>").setDescription("Path to output file to store the pose") ;

    try {
        args.parse(argc, (const char **)argv)  ;

        if ( print_help ) {
            args.printUsage(std::cout) ;
            exit(0);
        }
    } catch ( ArgumentParserException &e ) {
        cerr << e.what() << endl ;
        args.printUsage(std::cerr) ;
        exit(1) ;
    }

    CameraCalibration calib ;
    CameraCalibration::Data cdata ;

    if ( compute_markers ) {
        auto files = Path::entries(data_folder, DirectoryFilters::Glob("*.png"), false) ;

        AprilTagDetector adet(AprilTag36h11, 4) ; // we need decimation othwerwise detection does not work

        AprilTagGridPattern agrid(pattern_grid, pattern_width, pattern_gap, adet) ;

        calib.detect(files, agrid, cdata) ;

        cdata.save(data_folder + "/calib.data") ;
    }
    else
        cdata.load(data_folder + "/calib.data") ;

    PinholeCamera cam ;

    if  ( !cam.read(camera_intrinsics) ) {
        cerr << "can't read intrinsics from: " << camera_intrinsics << endl ;
        exit(1) ;
    }


  //  sensor_to_base.linear() << 0, 1, 0, 1, 0, 0, 0, 0, -1 ;
 //   sensor_to_base.linear() << 0, 1, 0, 1, 0, 0, 0, 0, -1 ;
//    sensor_to_base.translation() << -1, 0, -0.9 ;

    Matrix4d init ;
    init << 0.0, -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, -1.4,
            0.0, 0.0, -1.0, 2.45,
            0.0, 0.0, 0.0, 1.0 ;

    Affine3d sensor_to_base(init) ;
   sensor_to_base = sensor_to_base.inverse() ;

    vector<Affine3d> gripper_to_base, target_to_sensor ;
    Vector3d offset ;

    loadPoses(cam, cdata, target_to_sensor, gripper_to_base) ;
    bundleAdjustment(cam, cdata, gripper_to_base, sensor_to_base, offset) ;

    ofstream strm(output_file) ;
    strm << sensor_to_base.matrix() << endl ;

   drawResiduals(cam, cdata, gripper_to_base, sensor_to_base, cdata.coords_);

}

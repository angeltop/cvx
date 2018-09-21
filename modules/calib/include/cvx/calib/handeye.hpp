#ifndef __HANDEYE_HPP__
#define __HANDEYE_HPP__

#include <vector>
#include <string>
#include <Eigen/Geometry>

namespace cvx { namespace camera {

class HandEyeCalibration {
public:

    enum Method { DualQuaternion, Tsai, Horaud } ;

    struct Parameters {
        Method method_ = DualQuaternion ;
        bool refine_ = true ;
    };

    HandEyeCalibration() = default ;
    HandEyeCalibration(const Parameters &params): params_(params) {}


    // fixed camera moving target
    bool solveFixed(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                      Eigen::Affine3d &sensor_to_base) ;

    // moving camera fixed target
    bool solveMoving(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                      Eigen::Affine3d &sensor_to_gripper) ;


private:

    // method of Horaud and Dornaika
    bool solveHandEyeLinearHD(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X) ;
    // method of Tsai
    bool solveHandEyeLinearTsai(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X) ;
    // method by Danilidis
    bool solveHandEyeLinearDualQuaternion(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X) ;

    // Levenberg Marquard solver
    bool solveNonLinear(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X) ;

    void sortStationMovements(std::vector<Eigen::Affine3d> &gripper_to_base, std::vector<Eigen::Affine3d> &target_to_sensor) ;

private:

    Parameters params_ ;
} ;


}}

#endif

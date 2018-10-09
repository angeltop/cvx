#ifndef __CVX_VIZ_VIEWPOINT_SAMPLER_HPP__
#define __CVX_VIZ_VIEWPOINT_SAMPLER_HPP__

#include <vector>
#include <Eigen/Core>
#include <cvx/util/camera/camera.hpp>

namespace cvx { namespace util {

// Create viewpoints along a sphere e.g. to render an object from different angles
// The center of coordinates where the object is located is assumed to be Y-up, Z-towards the user

class ViewPointSampler {
public:

    ViewPointSampler() ;

    // set camera roll around z-axis
    void setRoll(float min_roll, float max_roll, float roll_step) ;
    // set distance range of camera from center
    void setRadius(float min_rad, float max_rad, float rad_step) ;
    // set center of sampling sphere ( default is (0, 0, 0)
    void setCenter(const Eigen::Vector3f &center) ;
    // set min-max altitude in radians ( by default only the positive elevations are returned [0,pi/2] )
    void setAltitude(float min_alt, float max_alt) ;
    // set min-max azimuth in radians
    void setAzimuth(float min_az, float max_az) ;

    // Based on "Minimal discrete energy on the sphere, E. A. Rakhmanov, E. B. Saff, and Y. M. Zhou"
    void generate(uint npts, std::vector<Eigen::Matrix4f> &views) ;

    // write collada file with all cameras
    static void exportCameras(const std::string &fname, const std::vector<Eigen::Matrix4f> &views, const PinholeCamera &cam, const std::string &id = "camera");

private:

    float min_roll_, max_roll_, roll_step_ ;
    float min_radius_, max_radius_, radius_step_ ;
    float min_altitude_, max_altitude_ ;
    float min_azimuth_, max_azimuth_ ;

    Eigen::Vector3f center_ ;
} ;

}}







#endif

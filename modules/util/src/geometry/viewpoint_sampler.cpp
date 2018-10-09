#include <cvx/util/geometry/viewpoint_sampler.hpp>
#include <cvx/util/geometry/util.hpp>
#include <cvx/util/misc/strings.hpp>

#include <Eigen/Geometry>
#include <fstream>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace util {

static Matrix4f lookAt(const Vector3f &eye, const Vector3f &center, float roll)
{
    Matrix4f lr = cvx::util::lookAt(eye, center, Vector3f(0, 1, 0)) ;

    Affine3f rot ;
    rot.setIdentity();
    rot.rotate(AngleAxisf(roll, Eigen::Vector3f::UnitZ())) ;
    return lr * rot.matrix() ;

}

// MINIMAL DISCRETE ENERGY ON THE SPHERE
// E. A. Rakhmanov, E. B. Saff, and Y. M. Zhou

static void generate_points_on_sphere(vector<Vector3f> &pts, uint N)
{
    float phi = 0.0 ;

    for(int k=0 ; k<N-1 ; k++)
    {
        float h = -1 + 2*k/(double)(N-1) ;
        float theta = acos(h) ;

        if ( k > 0 )
            phi = phi + 3.6/sqrt(N)/sqrt(1 - h*h) ;

        float x = std::cos(phi) * std::sin(theta);
        float y = std::sin(phi) * std::sin(theta);
        float z = std::cos(theta);

        pts.push_back(Vector3f(x, z, y)) ;
    }

    // singularity
    //    pts.push_back(Vector3f(0, 1, 0)) ;
}


void ViewPointSampler::generate(uint nPts, vector<Matrix4f> &views)
{
    vector<Vector3f> pts ;
    generate_points_on_sphere(pts, nPts) ;

    for(float roll = min_roll_ ; roll <= max_roll_ ; roll += roll_step_)
        for(float radius = min_radius_ ; radius <= max_radius_ ; radius += radius_step_)
            for(int i=0 ; i<pts.size() ; i++)
            {
                Vector3f o = pts[i] ;
                Vector3f p = o * radius ;

                float az = atan2(o.z(), o.x()) ;
                float el = atan2(o.y(), sqrt(o.x()*o.x() + o.z()*o.z())) ;

                if ( el > max_altitude_ ) continue ;
                if ( el < min_altitude_ ) continue ;
                if ( az > max_azimuth_ ) continue ;
                if ( az < min_azimuth_ ) continue ;

                Matrix4f lr = lookAt(p, center_, roll) ;

                views.push_back(lr) ;
            }
}

ViewPointSampler::ViewPointSampler()
{
    min_altitude_ = 0 ; max_altitude_ = M_PI/2 ;
    min_azimuth_ = -M_PI ; max_azimuth_ = M_PI ;
    min_roll_ = 0 ; max_roll_ = 0 ; roll_step_ = M_PI ;
    min_radius_ = 1 ; max_radius_ = 1 ; radius_step_ = 1 ;
    center_ << 0, 0, 0 ;
}

void ViewPointSampler::setRoll(float min_roll, float max_roll, float roll_step)
{
    min_roll_ = min_roll ; max_roll_ = max_roll ; roll_step_ = roll_step ;
    assert(min_roll <= max_roll) ;
}

void ViewPointSampler::setRadius(float min_rad, float max_rad, float rad_step)
{
    min_radius_ = min_rad ; max_radius_ = max_rad ; radius_step_ = rad_step ;
    assert(min_rad <= max_rad) ;
}

void ViewPointSampler::setAltitude(float min_alt, float max_alt)
{
    min_altitude_ = min_alt ; max_altitude_ = max_alt ;
    assert(min_alt <= max_alt) ;
}

void ViewPointSampler::setAzimuth(float min_az, float max_az)
{
    min_azimuth_ = min_az ; max_azimuth_ = max_az ;
    assert(min_az <= max_az) ;
}

void ViewPointSampler::setCenter(const Vector3f &center) {
    center_ = center ;
}



void ViewPointSampler::exportCameras(const string &fname, const std::vector<Matrix4f> &views, const PinholeCamera &cam, const string &cam_id) {

    ofstream strm(fname.c_str()) ;

    strm << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
            "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n"
            "<asset>\n"
            "\t<unit name=\"meter\" meter=\"1\"/>\n"
            "\t<up_axis>Y_UP</up_axis>\n"
            "</asset>\n"  ;

    strm << "<library_cameras>\n"  ;

    float xfov = 2 * atan( cam.sz().width / cam.fx()/2.0)  ;
    float aspect = cam.sz().width / (float) cam.sz().height ;

    for( uint i=0 ; i<views.size() ; i++ ) {
        string id = format("%s-%04d-camera", cam_id, i) ;
        strm << "\t<camera id=\"" << id << "\" name=\"" << id << "\">\n" ;
        strm << "\t\t<optics>\n" ;
        strm << "\t\t\t<technique_common>\n" ;
        strm << "\t\t\t\t<perspective>\n" ;
        strm << "\t\t\t\t\t<xfov sid=\"xfov\">" << xfov * 180 / M_PI << "</xfov>\n" ;
        strm << "\t\t\t\t\t<aspect_ratio>" << aspect << "</aspect_ratio>\n" ;
        strm << "\t\t\t\t\t<znear sid=\"znear\">" << 0.001 << "</znear>\n" ;
        strm << "\t\t\t\t\t<zfar sid=\"zfar\">" << 100.0 << "</zfar>\n" ;
        strm << "\t\t\t\t</perspective>\n" ;
        strm << "\t\t\t</technique_common>\n" ;
        strm << "\t\t</optics>\n" ;
        strm << "\t</camera>\n" ;
    }
    strm << "</library_cameras>\n" ;

    strm << "<library_visual_scenes>\n" ;
    strm << "\t<visual_scene id=\"Scene\" name=\"Scene\">\n" ;

    for( uint i=0 ; i<views.size() ; i++ ) {
        string id = format("%s-%04d", cam_id, i) ;
        Matrix4f mat = views[i].inverse() ;
        strm << "\t\t<node id=\"" << id << "\" name=\"" << id << "\" type=\"NODE\">\n" ;
        strm << "\t\t\t<matrix sid=\"transform\">" ;
        strm << mat ;
        strm << "</matrix>\n" ;
        strm << "\t\t\t<instance_camera url=\"#" << id << "-camera\"/>\n" ;
        strm << "\t\t</node>\n" ;
    }

    strm << "\t</visual_scene>\n"
            "</library_visual_scenes>\n"
            "<scene>\n"
            "\t<instance_visual_scene url=\"#Scene\"/>\n"
            "</scene>\n"
            "</COLLADA>" ;
}


}}

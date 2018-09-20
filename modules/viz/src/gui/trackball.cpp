#include <cvx/viz/gui/trackball.hpp>

#include <cvx/viz/scene/scene.hpp>
#include <cvx/viz/scene/camera.hpp>

#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;
using namespace cvx ;

Eigen::Vector3f TrackBall::X(1.f, 0.f, 0.f);
Eigen::Vector3f TrackBall::Y(0.f, 1.f, 0.f);
Eigen::Vector3f TrackBall::Z(0.f, 0.f, 1.f);

TrackBall::TrackBall() :
    camera_motion_left_click_(ARC),
    camera_motion_middle_click_(ROLL),
    camera_motion_right_click_(PAN),
    camera_motion_scroll_(ZOOM),
    height_(1),
    is_dragging_(false),
    is_left_click_(false),
    is_middle_click_(false),
    is_right_click_(false),
    is_scrolling_(false),
    pan_scale_(.005f),
    roll_scale_(.005f),
    roll_sum_(0.f),
    rotation_(1.f, 0, 0, 0),
    rotation_sum_(1.f, 0, 0, 0),
    speed_(1.f),
    width_(1),
    zoom_scale_(20.f),
    zoom_sum_(0.f)
{
}

TrackBall::~TrackBall()
{
}


char TrackBall::clickQuadrant(float x, float y)
{
    float halfw = .5 * width_;
    float halfh = .5 * height_;

    if (x > halfw) {
        // Opengl image coordinates origin is upperleft.
        if (y < halfh) {
            return 1;
        } else {
            return 4;
        }
    } else {
        if (y < halfh) {
            return 2;
        } else {
            return 3;
        }
    }
}

Vector3f TrackBall::computeCameraEye()
{
    Vector3f orientation = rotation_sum_ * Z;

    if (zoom_sum_) {
        translation_length_ += zoom_scale_ * zoom_sum_;
        zoom_sum_ = 0; // Freeze zooming after applying.
    }

    return  translation_length_ * orientation + center_ ;
}

Vector3f TrackBall::computeCameraUp()
{
    return (rotation_sum_ * Y).normalized() ;
}

Vector3f TrackBall::computePan()
{
    float dx = cx_ - pcx_, dy = cy_ - pcy_;
    Vector3f look = eye_ - center_ ;
    float length = look.norm() ;
    Vector3f right = (rotation_sum_ * X).normalized() ;

    return ( up_ * -dy + right * dx ) * pan_scale_ * speed_ * length ;
}

Vector3f TrackBall::computePointOnSphere(float px, float py) {
    // https://www.opengl.org/wiki/Object_Mouse_Trackball
    float x = (2.f * px - width_) / width_;
    float y = (height_ - 2.f * py) / height_;

    float length2 = x*x + y*y;

    Vector3f result ;

    if (length2 <= .5) {
        result.z() = sqrt(1.0 - length2);
    } else {
        result.z() = 0.5 / sqrt(length2);
    }

    float norm = 1.0 / sqrt(length2 + result.z()*result.z());

    result.x() = x * norm;
    result.y() = y * norm;
    result.z() *= norm;

    return result ;
}

Quaternionf TrackBall::computeRotationBetweenVectors(const Vector3f &u, const Vector3f &v) {
    float cosTheta = u.dot(v) ;
    Vector3f rotationAxis ;
    static const float EPSILON = 1.0e-5f;

    Quaternionf result ;

 	if (cosTheta < -1.0f + EPSILON){
 		// Parallel and opposite directions.
        rotationAxis = Vector3f(0.f, 0.f, 1.f).cross(u) ;

        if ( rotationAxis.squaredNorm() < 0.01 ) {
            // Still parallel, retry.
            rotationAxis = Vector3f(1.f, 0.f, 0.f).cross(u);
        }

        rotationAxis.normalize() ;
        result = AngleAxisf(M_PI, rotationAxis);
 	} else if (cosTheta > 1.0f - EPSILON) {
        // Parallel and same direction.
        return Quaternionf(1, 0, 0, 0);
    } else {
        float theta = acos(cosTheta);
        rotationAxis = u.cross(v).normalized();

        result = AngleAxisf(theta * speed_, rotationAxis);
    }

    return result ;

}

void TrackBall::drag()
{
    if ( cx_ == pcx_ && cy_ == pcy_ ) {
        // Not moving during drag state, so skip unnecessary processing.
        return;
    }

    stop_ = computePointOnSphere(cx_, cy_);
    rotation_ = computeRotationBetweenVectors(start_, stop_);
    // Reverse so scene moves with cursor and not away due to camera model.
    rotation_ = rotation_.inverse();

    drag(is_left_click_, camera_motion_left_click_);
    drag(is_middle_click_, camera_motion_middle_click_);
    drag(is_right_click_, camera_motion_right_click_);

    // After applying drag, reset relative start state.
    pcx_ = cx_ ; pcy_ = cy_ ;
    start_ = stop_ ;
}

void TrackBall::drag(bool isClicked, CameraMotionType motion)
{
    if (!isClicked) {
        return;
    }

    switch(motion) {
        case ARC:
            dragArc();
            break;
        case FIRSTPERSON:
            dragFirstPerson();
            break;
        case PAN:
            dragPan();
            break;
        case ROLL:
            rollCamera();
            break;
        case ZOOM:
            dragZoom();
            break;
        default: break;
    }
}

void TrackBall::dragArc()
{
    rotation_sum_ *= rotation_; // Accumulate quaternions.

    updateCameraEyeUp(true, true);
}

void TrackBall::dragFirstPerson()
{
    Vector3f pan = computePan();

    center_ = pan + center_ ;
    camera_->lookAt(eye_, center_, up_) ;

    freezeTransform();
}

void TrackBall::dragPan()
{
    Vector3f pan = computePan();

    center_ = pan + center_ ;
    eye_ = pan + eye_ ;

    camera_->lookAt(eye_, center_, up_) ;

    freezeTransform();
}

void TrackBall::dragZoom()
{
    float dirx = cx_ - pcx_, diry = cy_ - pcy_ ;
    float ax = fabs(dirx);
    float ay = fabs(diry);

    if (ay >= ax) {
        setScrollDirection(diry <= 0);
    } else {
        setScrollDirection(dirx <= 0);
    }

    updateCameraEyeUp(true, false);
}

void TrackBall::freezeTransform()
{
    if (camera_) {
        // Opengl is ZYX order.
        // Flip orientation to rotate scene with sticky cursor.

        Affine3f m(camera_->getViewMatrix()) ;
        Quaternionf cr(m.rotation()) ;
        rotation_sum_ = cr.inverse();
        translation_length_ = (eye_ - center_).norm();
    }
}


TrackBall::CameraMotionType TrackBall::getMotionLeftClick()
{
    return camera_motion_left_click_;
}

TrackBall::CameraMotionType TrackBall::getMotionMiddleClick()
{
    return camera_motion_middle_click_;
}

TrackBall::CameraMotionType TrackBall::getMotionRightClick()
{
    return camera_motion_right_click_;
}

TrackBall::CameraMotionType TrackBall::getMotionScroll()
{
    return camera_motion_scroll_;
}

void TrackBall::rollCamera()
{
    float deltax = cx_ - pcx_, deltay = cy_ - pcy_ ;
    char quad = clickQuadrant(cx_, cy_);
    switch (quad) {
        case 1:
            deltay = -deltay;
            deltax = -deltax;
            break;
        case 2:
            deltax = -deltax;
            break;
        case 3:
            break;
        case 4:
            deltay = -deltay;
        default:
            break;
    }

    Vector3f axis = (center_ - up_).normalized();
    float angle = roll_scale_ * speed_ * (deltax + deltay + roll_sum_);
    Quaternionf rot ;
    rot = AngleAxisf(angle, axis);
    up_ = rot * up_ ;
    camera_->lookAt(eye_, center_, up_) ;
    freezeTransform();
    roll_sum_ = 0;
}

void TrackBall::scroll()
{
    switch(camera_motion_scroll_) {
        case ROLL:
            rollCamera();
            break;
        case ZOOM:
            updateCameraEyeUp(true, false);
            break;
        default: break;
    }
}

void TrackBall::setCamera(CameraPtr c, const Vector3f &eye, const Vector3f &center, const Vector3f &up) {
    camera_ = c;
    eye_ = eye ; center_ = center ; up_ = up ;
    camera_->lookAt(eye, center, up) ;
    freezeTransform();
}

void TrackBall::setZoomScale(float zoom_scale)
{
    zoom_scale_ = zoom_scale ;
}

void TrackBall::setClickPoint(double x, double y) {
    pcx_ = cx_ ; pcy_ = cy_ ;
    cx_ = x ; cy_ = y ;
}

void TrackBall::setLeftClicked(bool value) {
    is_left_click_ = value;
}

void TrackBall::setMiddleClicked(bool value) {
    is_middle_click_ = value;
}

void TrackBall::setMotionLeftClick(CameraMotionType motion) {
    camera_motion_left_click_ = motion;
}

void TrackBall::setMotionMiddleClick(CameraMotionType motion) {
    camera_motion_middle_click_ = motion;
}

void TrackBall::setMotionRightClick(CameraMotionType motion) {
    camera_motion_right_click_ = motion;
}

void TrackBall::setMotionScroll(CameraMotionType motion) {
    camera_motion_scroll_ = motion;
}

void TrackBall::setRightClicked(bool value) {
    is_right_click_ = value;
}

void TrackBall::setScreenSize(float width, float height) {
    if (width > 1 && height > 1) {
        width_ = width;
        height_ = height;
    }
}

void TrackBall::setScrollDirection(bool up) {
    is_scrolling_ = true;
    float inc = speed_ * (up ? -1.f : 1.f);
    zoom_sum_ += inc;
    roll_sum_ += inc;
}

void TrackBall::setSpeed(float s) {
    speed_ = s;
}

void TrackBall::update() {
    const bool isClick = is_left_click_ || is_middle_click_ || is_right_click_;

    if (! is_dragging_)
    {
        if (isClick)
        {
            is_dragging_ = true;
            start_ = computePointOnSphere(cx_, cy_);
        } else if (is_scrolling_) {
            scroll();
            is_scrolling_ = false;
        }
    } else
    {
        if (isClick)
        {
            drag();
        } else
        {
            is_dragging_ = false;
        }
    }
}

void TrackBall::updateCameraEyeUp(bool eye, bool up)
{
    if (eye)
        eye_ = computeCameraEye() ;

    if (up)
        up_ = computeCameraUp() ;

    camera_->lookAt(eye_, center_, up_);
}


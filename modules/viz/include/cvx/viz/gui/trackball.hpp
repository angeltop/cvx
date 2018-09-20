#ifndef __TRACKBALL_HPP__
#define __TRACKBALL_HPP__

#include <cvx/viz/scene/scene_fwd.hpp>
#include <Eigen/Geometry>

using cvx::Camera ;

// adapted from https://github.com/rsmz/trackball/

class TrackBall
{
public:

    TrackBall();
    ~TrackBall();

    enum CameraMotionType {
        NONE, ARC, FIRSTPERSON, PAN, ROLL, ZOOM
    } ;

    static Eigen::Vector3f X, Y, Z ;

    Eigen::Vector3f computePointOnSphere(float px, float py);
    Eigen::Quaternionf computeRotationBetweenVectors(const Eigen::Vector3f & start,  const Eigen::Vector3f& stop) ;

    CameraMotionType getMotionLeftClick();
    CameraMotionType getMotionMiddleClick();
    CameraMotionType getMotionRightClick();
    CameraMotionType getMotionScroll();

    void setScrollDirection(bool up);
    void setCamera(cvx::CameraPtr c, const Eigen::Vector3f &eye, const Eigen::Vector3f &center, const Eigen::Vector3f &up);
    void setZoomScale(float zoom_scale) ;
	void setClickPoint(double x, double y);
	void setLeftClicked(bool value);
	void setMiddleClicked(bool value);
	void setMotionLeftClick(CameraMotionType motion);
	void setMotionMiddleClick(CameraMotionType motion);
	void setMotionRightClick(CameraMotionType motion);
    void setMotionScroll(CameraMotionType motion);
	void setRightClicked(bool value);
	void setScreenSize(float width, float height);
	void setSpeed(float s);
	void update();

protected:
    char clickQuadrant(float x, float y);
    Eigen::Vector3f computeCameraEye();
    Eigen::Vector3f computeCameraUp();
    Eigen::Vector3f computePan();
    void drag();
    void drag(bool isClicked, CameraMotionType motion);
    void dragArc();
    // Change eye position and up direction while keeping center point static.
    void dragArcCamera();
    void dragFirstPerson();
    void dragZoom();
    // Simulate zoom by moving camera along viewing eye direction.
    void dragZoomCamera();
    // Move camera focal center position with static up and eye direction.
    void dragPan();
    // Roll about eye direction.
    void rollCamera();
    void freezeTransform();
    void scroll();
    void updateCameraEyeUp(bool eye, bool up);

private:
    cvx::CameraPtr camera_;
    CameraMotionType camera_motion_left_click_;
    CameraMotionType camera_motion_middle_click_;
    CameraMotionType camera_motion_right_click_;
    CameraMotionType camera_motion_scroll_;
    float cx_, cy_, pcx_, pcy_;
    float height_, pan_scale_;
    bool is_dragging_, is_left_click_, is_middle_click_, is_right_click_, is_scrolling_ ;
    float roll_scale_, roll_sum_, speed_ ;

    Eigen::Quaternionf rotation_, rotation_sum_ ;
    Eigen::Vector3f start_, stop_ ;
    float translation_length_, width_, zoom_sum_, zoom_scale_ ;
    Eigen::Vector3f center_, eye_, up_ ;

};

#endif

/*
    LICENSE BEGIN

    trackball - A 3D view interactor for C++ programs.
    Copyright (C) 2016  Remik Ziemlinski

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    LICENSE END
*/

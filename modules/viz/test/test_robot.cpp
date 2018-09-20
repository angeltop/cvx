#include <cvx/viz/renderer/renderer.hpp>

#include <cvx/viz/robot/robot_scene.hpp>
#include <cvx/viz/scene/camera.hpp>
#include <cvx/viz/scene/light.hpp>
#include <cvx/viz/scene/node.hpp>

#include <cvx/viz/gui/glfw_window.hpp>
#include <cvx/viz/gui/trackball.hpp>

#include <iostream>
#include <thread>
using namespace cvx::viz ;

using namespace std ;
using namespace Eigen ;

class glfwGUI: public glfwRenderWindow {
public:

    glfwGUI(RobotScenePtr scene): glfwRenderWindow(), scene_(scene), rdr_(scene) {
        auto c = scene->geomCenter() ;
        auto r = scene->geomRadius(c) ;

        camera_.reset(new PerspectiveCamera(1.0, 50*M_PI/180, 0.001, 10*r)) ;
        trackball_.setCamera(camera_, c + Vector3f{0.0, 0, 2*r}, c, {0, 1, 0}) ;
        trackball_.setZoomScale(0.1*r) ;

        palm_joint_ = std::dynamic_pointer_cast<RevoluteJoint>(scene_->getJoint("palm_right_finger")) ;
        palm_joint_pos_ = 0.0 ;

        subfinger_joint_ = std::dynamic_pointer_cast<RevoluteJoint>(scene_->getJoint("left_finger_left_subfinger")) ;
        subfinger_pos_ = 0.0 ;
    }

    void onInit() {
        rdr_.init() ;

    }

    void onResize(int width, int height) {
        float ratio;
        ratio = width / (float) height;

        trackball_.setScreenSize(width, height);

        static_pointer_cast<PerspectiveCamera>(camera_)->setAspectRatio(ratio) ;

        camera_->setViewport(width, height)  ;
    }


    void onKeyPressed(size_t key, uint) override {
        if (key == GLFW_KEY_Q ) {
            palm_joint_pos_ -= 0.1 ;
            palm_joint_pos_ = palm_joint_->setPosition(palm_joint_pos_) ;

        } else if ( key ==  GLFW_KEY_W ) {
            palm_joint_pos_ += 0.1 ;
            palm_joint_pos_ = palm_joint_->setPosition(palm_joint_pos_) ;
        } else if (key == GLFW_KEY_A ) {
            subfinger_pos_ = subfinger_joint_->setPosition(subfinger_pos_ - 0.1) ;
        } else if ( key ==  GLFW_KEY_S ) {
            subfinger_pos_ = subfinger_joint_->setPosition(subfinger_pos_ + 0.1) ;
        }
    }

    void onMouseButtonPressed(uint button, size_t x, size_t y, uint flags) override {
        switch ( button ) {
            case GLFW_MOUSE_BUTTON_LEFT:
                trackball_.setLeftClicked(true) ;
                break ;
            case GLFW_MOUSE_BUTTON_MIDDLE:
                trackball_.setMiddleClicked(true) ;
                break ;
            case GLFW_MOUSE_BUTTON_RIGHT:
                trackball_.setRightClicked(true) ;
                break ;
        }
        trackball_.setClickPoint(x, y) ;
    }

    void onMouseButtonReleased(uint button, size_t x, size_t y, uint flags) override {
        switch ( button ) {
            case GLFW_MOUSE_BUTTON_LEFT:
                trackball_.setLeftClicked(false) ;
                break ;
            case GLFW_MOUSE_BUTTON_MIDDLE:
                trackball_.setMiddleClicked(false) ;
                break ;
            case GLFW_MOUSE_BUTTON_RIGHT:
                trackball_.setRightClicked(false) ;
                break ;
        }

    }

    void onMouseMoved(double xpos, double ypos) override {
        ostringstream s ;
        s << xpos << ',' << ypos ;
        text_ = s.str() ;

        trackball_.setClickPoint(xpos, ypos) ;
    }

    void onMouseWheel(double x) {
        trackball_.setScrollDirection(x>0);
    }


    void onRender() {
        trackball_.update() ;
        rdr_.render(camera_) ;
        this_thread::yield() ;
    }

    string text_ ;
    Renderer rdr_ ;
    RobotScenePtr scene_ ;
    TrackBall trackball_ ;
    CameraPtr camera_ ;
    std::shared_ptr<RevoluteJoint> palm_joint_, subfinger_joint_ ;
    float palm_joint_pos_, subfinger_pos_ ;

};

class HierarchyPrinter: public NodeVisitor {
public:
    void visit(Node &node) override {
        for( uint i=0 ; i<count_ ; i++ )
            cout << '\t' ;
        string name = node.name() ;
        if ( name.empty() )
            cout << "<empty>" << endl ;
        else
            cout << node.name() << endl ;

        for( uint i=0 ; i<count_ ; i++ )  cout << '\t' ;
       cout << node.globalTransform().translation().adjoint() << endl ;
         // cout << node.matrix().translation().adjoint() << endl ;
        for( uint i=0 ; i<count_ ; i++ )  cout << '\t' ;
        cout << Quaternionf(node.globalTransform().rotation()).coeffs().adjoint() << endl ;
     //      cout << Quaternionf(node.matrix().rotation()).coeffs().adjoint() << endl ;

        count_++ ;
        visitChildren(node) ;
        count_-- ;
    }

    size_t count_ = 0 ;
};

int main(int argc, char *argv[]) {

//   string package_path = "/home/malasiot/source/radioroso_ws/src/radioroso_certh/cvx_grasp_planner/" ;
//    RobotScenePtr scene = RobotScene::loadURDF(package_path + "urdf/radioroso-gripper.urdf", { { "cvx_grasp_planner", package_path } }) ;

    string package_path = "/home/malasiot/source/radioroso_ws/src/clopema_testbed/clopema_description/" ;
    RobotScenePtr scene = RobotScene::loadURDF(package_path + "robots/clopema.urdf", { { "clopema_description", package_path } }, true) ;

    vector<string> hidden_nodes = { "ctu_f_roof_1", "ctu_f_roof_2", "ctu_f_roof_3", "ctu_f_wall_1",
                                    "ctu_f_wall_2", "ctu_f_wall_3", "ctu_f_wall_4" };

    for( const auto &s: hidden_nodes )
        scene->findNodeByName(s)->setVisible(false) ;

    HierarchyPrinter pr ;
    pr.visit(*scene);
    DirectionalLight *dl = new DirectionalLight(Vector3f(0.5, 0.5, 1)) ;
    dl->diffuse_color_ = Vector3f(1, 1, 1) ;
    scene->addLight(LightPtr(dl)) ;

    glfwGUI gui(scene) ;

    gui.run(500, 500) ;

}

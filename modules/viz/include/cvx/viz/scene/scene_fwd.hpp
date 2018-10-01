#ifndef __VSIM_SCENE_FWD_HPP__
#define __VSIM_SCENE_FWD_HPP__

#include <memory>

namespace cvx { namespace viz {

struct Material ;
typedef std::shared_ptr<Material> MaterialPtr ;

struct Mesh ;
typedef std::shared_ptr<Mesh> MeshPtr ;

class Geometry ;
typedef std::shared_ptr<Geometry> GeometryPtr ;

struct Camera ;
typedef std::shared_ptr<Camera> CameraPtr ;

struct Light ;
typedef std::shared_ptr<Light> LightPtr ;

class Scene ;
typedef std::shared_ptr<Scene> ScenePtr ;

class Drawable ;
typedef std::shared_ptr<Drawable> DrawablePtr ;

class Node ;
typedef std::shared_ptr<Node> NodePtr ;

}}

#endif

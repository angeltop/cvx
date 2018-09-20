#ifndef __SCENE_IMPORTER_HPP__
#define __SCENE_IMPORTER_HPP__

#include <string>
#include <stdexcept>
#include <vector>
#include <memory>

#include <cvx/viz/renderer/scene.hpp>

namespace cvx { namespace viz {

class SceneLoader ;
typedef std::shared_ptr<SceneLoader> SceneLoaderPtr ;

class SceneLoaderException: public std::runtime_error {

public:

    SceneLoaderException(const std::string &driver_name, const std::string &message, const std::string &fname, int line = -1):
        std::runtime_error(driver_name + ":" + message + "(" + fname + ((line > 0) ? (":" + std::to_string(line)) : "") + ")") {}
};

} // namespace renderer
} // namespace vsim

#endif

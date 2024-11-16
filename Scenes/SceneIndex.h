#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "Scene4.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Complex Simulation", creator<Scene4>()}
    // add more Scene types here
};

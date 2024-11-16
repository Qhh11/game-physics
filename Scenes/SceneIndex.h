#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "Scene3.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<Scene1>()},
    {"Midpoint Simulation", creator<Scene3>()},
};

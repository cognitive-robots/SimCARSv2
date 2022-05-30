#pragma once

#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IFileBasedScene : public virtual IScene
{
public:
    virtual void save(const std::string& output_file_path_str) const = 0;
};

}
}
}

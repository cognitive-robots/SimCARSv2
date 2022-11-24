#pragma once

#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ITwoFileBasedScene : public virtual IScene
{
public:
    virtual void save(std::string const &output_file_1_path_str, std::string const &output_file_2_path_str) const = 0;
};

}
}
}

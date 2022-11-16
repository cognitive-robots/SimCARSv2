#pragma once

#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class AScene : public virtual IScene
{
public:
    structures::IArray<IValuelessConstant const*>* get_constants() const override;
    IValuelessConstant const* get_constant(std::string const &constant_name) const override;

    structures::IArray<IValuelessVariable const*>* get_variables() const override;
    IValuelessVariable const* get_variable(std::string const &variable_name) const override;

    structures::IArray<IValuelessEvent const*>* get_events() const override;
};

}
}
}

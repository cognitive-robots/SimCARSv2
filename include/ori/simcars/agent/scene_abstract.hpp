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
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constants() const override;
    std::shared_ptr<const IValuelessConstant> get_constant(const std::string& constant_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variables() const override;
    std::shared_ptr<const IValuelessVariable> get_variable(const std::string& variable_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const override;
    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events(temporal::Time time) const override;
};

}
}
}

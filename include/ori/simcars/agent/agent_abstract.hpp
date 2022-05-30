#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/agent_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class AAgent : public IAgent, public std::enable_shared_from_this<AAgent>
{
public:
    std::shared_ptr<const geometry::ORect> get_collision_box(temporal::Time timestamp) const override;
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> get_other_agents() const override;
    bool operator ==(const IAgent& agent) const override;
};

}
}
}

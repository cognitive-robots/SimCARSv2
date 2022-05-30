#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/living_interface.hpp>
#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/agent_state_holder_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>

#include <memory>

namespace ori
{
namespace simcars
{
namespace agent
{

class IAgent : public IAgentStateHolder, public temporal::ILiving<IAgentStateHolder>
{
public:
    virtual uint32_t get_id() const = 0;
    virtual bool is_ego() const = 0;
    virtual bool is_ever_simulated() const = 0;
    virtual bool is_simulated(temporal::Time timestamp) const = 0;
    virtual FP_DATA_TYPE get_length() const = 0;
    virtual FP_DATA_TYPE get_width() const = 0;
    virtual std::shared_ptr<const geometry::ORect> get_collision_box(temporal::Time timestamp) const = 0;
    virtual IAgent::Class get_class() const = 0;
    virtual std::shared_ptr<const IScene> get_scene() const = 0;
    virtual std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> get_other_agents() const = 0;
    virtual bool operator ==(const IAgent& agent) const = 0;
};

}
}
}

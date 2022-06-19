#pragma once

#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/agent/controller_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingAgentController : public virtual IController
{
    std::shared_ptr<const IEntity> entity;
    std::shared_ptr<const map::IMap<std::string>> map;

public:
    DrivingAgentController(std::shared_ptr<const IEntity> entity, std::shared_ptr<const map::IMap<std::string>> map);

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_indirect_actuation_events(temporal::Time time) const override;
};

}
}
}

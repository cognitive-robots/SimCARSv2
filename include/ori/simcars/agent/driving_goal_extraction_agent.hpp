#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingGoalExtractionAgent : public virtual ADrivingAgent
{
    std::string name;

    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessConstant>> constant_dict;
    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessVariable>> variable_dict;

    DrivingGoalExtractionAgent();

    void extract_aligned_linear_velocity_change_events();
    void extract_lane_change_events(std::shared_ptr<const map::IMap<std::string>> map);

public:
    DrivingGoalExtractionAgent(std::shared_ptr<const IDrivingAgent> driving_agent);
    DrivingGoalExtractionAgent(std::shared_ptr<const IDrivingAgent> driving_agent, std::shared_ptr<const map::IMap<std::string>> map);

    std::string get_name() const override;

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constant_parameters() const override;
    std::shared_ptr<const IValuelessConstant> get_constant_parameter(const std::string& constant_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variable_parameters() const override;
    std::shared_ptr<const IValuelessVariable> get_variable_parameter(const std::string& variable_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const override;

    std::shared_ptr<IDrivingAgent> driving_agent_deep_copy() const override;

    std::shared_ptr<const IDrivingAgentState> get_driving_agent_state(temporal::Time time) const override;
};

}
}
}

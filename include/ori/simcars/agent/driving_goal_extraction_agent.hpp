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
    IDrivingAgent const *driving_agent;

    structures::stl::STLDictionary<std::string, IValuelessVariable const*> variable_dict;

    DrivingGoalExtractionAgent();

    void extract_aligned_linear_velocity_change_events();
    void extract_lane_change_events(map::IMap<std::string> const *map);

public:
    DrivingGoalExtractionAgent(IDrivingAgent const *driving_agent);
    DrivingGoalExtractionAgent(IDrivingAgent const *driving_agent, map::IMap<std::string> const *map);

    ~DrivingGoalExtractionAgent();

    std::string get_name() const override;

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IValuelessConstant const*>* get_constant_parameters() const override;
    IValuelessConstant const* get_constant_parameter(std::string const &constant_name) const override;

    structures::IArray<IValuelessVariable const*>* get_variable_parameters() const override;
    IValuelessVariable const* get_variable_parameter(std::string const &variable_name) const override;

    structures::IArray<IValuelessEvent const*>* get_events() const override;

    IDrivingAgent* driving_agent_deep_copy() const override;

    IDrivingAgentState* get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const override;
};

}
}
}

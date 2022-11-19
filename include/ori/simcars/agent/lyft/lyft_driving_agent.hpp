#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>

#include <rapidjson/document.h>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

class LyftDrivingAgent : public virtual ADrivingAgent
{
    std::string name;

    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, IValuelessConstant const*> constant_dict;
    structures::stl::STLDictionary<std::string, IValuelessVariable const*> variable_dict;

    LyftDrivingAgent();

public:
    LyftDrivingAgent(rapidjson::Value::ConstObject const &json_agent_data);

    ~LyftDrivingAgent();

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
};

}
}
}
}

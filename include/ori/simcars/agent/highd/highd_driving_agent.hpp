#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>

#include <rapidcsv.h>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace highd
{

class HighDDrivingAgent : public virtual ADrivingAgent
{
    std::string name;

    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    structures::stl::STLDictionary<std::string, IValuelessConstant*> constant_dict;
    structures::stl::STLDictionary<std::string, IValuelessVariable*> variable_dict;

    HighDDrivingAgent();

public:
    HighDDrivingAgent(size_t tracks_meta_row, rapidcsv::Document const &tracks_meta_csv_document,
                      rapidcsv::Document const &tracks_csv_document);

    ~HighDDrivingAgent();

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


    structures::IArray<IValuelessConstant*>* get_mutable_constant_parameters() override;
    IValuelessConstant* get_mutable_constant_parameter(std::string const &constant_name) override;

    structures::IArray<IValuelessVariable*>* get_mutable_variable_parameters() override;
    IValuelessVariable* get_mutable_variable_parameter(std::string const &variable_name) override;

    structures::IArray<IValuelessEvent*>* get_mutable_events() override;
};

}
}
}
}

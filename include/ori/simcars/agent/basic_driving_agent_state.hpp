#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingAgentState : public virtual ADrivingAgentState
{
    std::string name;
    temporal::Time time;
    bool delete_dicts;

    void set_parameter_value(IValuelessConstant *parameter_value);

protected:
    structures::stl::STLDictionary<std::string, IValuelessConstant*> parameter_dict;

public:
    BasicDrivingAgentState(std::string const &name, temporal::Time time, bool delete_dicts = true);
    BasicDrivingAgentState(IReadOnlyDrivingAgentState const *driving_agent_state);
    BasicDrivingAgentState(IDrivingAgentState *driving_agent_state, bool copy_parameters = true);

    ~BasicDrivingAgentState();

    std::string get_name() const override;
    temporal::Time get_time() const override;

    bool is_populated() const override;

    structures::IArray<IValuelessConstant const*>* get_parameter_values() const override;
    IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const override;

    void set_id_constant(IConstant<uint32_t> *id_constant) override;
    void set_ego_constant(IConstant<bool> *ego_constant) override;
    void set_bb_length_constant(IConstant<FP_DATA_TYPE> *bb_length_constant) override;
    void set_bb_width_constant(IConstant<FP_DATA_TYPE> *bb_width_constant) override;
    void set_driving_agent_class_constant(IConstant<DrivingAgentClass> *driving_agent_class_constant) override;
    void set_position_variable(IConstant<geometry::Vec> *position_variable) override;
    void set_linear_velocity_variable(IConstant<geometry::Vec> *linear_velocity_variable) override;
    void set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> *aligned_linear_velocity_variable) override;
    void set_linear_acceleration_variable(IConstant<geometry::Vec> *linear_acceleration_variable) override;
    void set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> *aligned_linear_acceleration_variable) override;
    void set_external_linear_acceleration_variable(IConstant<geometry::Vec> *external_linear_acceleration_variable) override;
    void set_rotation_variable(IConstant<FP_DATA_TYPE> *rotation_variable) override;
    void set_steer_variable(IConstant<FP_DATA_TYPE> *steer_variable) override;
    void set_angular_velocity_variable(IConstant<FP_DATA_TYPE> *angular_velocity_variable) override;


    structures::IArray<IValuelessConstant*>* get_mutable_parameter_values() override;
    IValuelessConstant* get_mutable_parameter_value(std::string const &parameter_name) override;
};

}
}
}

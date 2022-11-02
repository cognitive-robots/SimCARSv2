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
    std::string driving_agent_name;

protected:
    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessConstant>> parameter_dict;

public:
    BasicDrivingAgentState(const std::string& driving_agent_name);
    BasicDrivingAgentState(std::shared_ptr<const IDrivingAgentState> driving_agent_state);

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_parameter_values() const override;
    std::shared_ptr<const IValuelessConstant> get_parameter_value(const std::string& parameter_name) const override;

    std::string get_driving_agent_name() const override;

    void set_driving_agent_name(const std::string& driving_agent_name) override;

    void set_id_constant(std::shared_ptr<const IConstant<uint32_t>> id_constant) override;
    void set_ego_constant(std::shared_ptr<const IConstant<bool>> ego_constant) override;
    void set_bb_length_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_length_constant) override;
    void set_bb_width_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_width_constant) override;
    void set_driving_agent_class_constant(std::shared_ptr<const IConstant<DrivingAgentClass>> driving_agent_class_constant) override;
    void set_position_variable(std::shared_ptr<const IConstant<geometry::Vec>> position_variable) override;
    void set_linear_velocity_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_velocity_variable) override;
    void set_aligned_linear_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_variable) override;
    void set_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_acceleration_variable) override;
    void set_aligned_linear_acceleration_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_acceleration_variable) override;
    void set_external_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec>> external_linear_acceleration_variable) override;
    void set_rotation_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> rotation_variable) override;
    void set_steer_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> steer_variable) override;
    void set_angular_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> angular_velocity_variable) override;
};

}
}
}

#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentState : public virtual IState
{
public:
    virtual std::string get_driving_agent_name() const = 0;

    virtual std::shared_ptr<const IConstant<uint32_t>> get_id_constant() const = 0;
    virtual std::shared_ptr<const IConstant<bool>> get_ego_constant() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_length_constant() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_width_constant() const = 0;
    virtual std::shared_ptr<const IConstant<DrivingAgentClass>> get_driving_agent_class_constant() const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_position_variable() const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_linear_velocity_variable() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_velocity_variable() const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_linear_acceleration_variable() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_acceleration_variable() const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_external_linear_acceleration_variable() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_rotation_variable() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_steer_variable() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_angular_velocity_variable() const = 0;

    virtual void set_driving_agent_name(const std::string& driving_agent_name) = 0;

    virtual void set_id_constant(std::shared_ptr<const IConstant<uint32_t>> id_constant) = 0;
    virtual void set_ego_constant(std::shared_ptr<const IConstant<bool>> ego_constant) = 0;
    virtual void set_bb_length_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_length_constant) = 0;
    virtual void set_bb_width_constant(std::shared_ptr<const IConstant<FP_DATA_TYPE>> bb_width_constant) = 0;
    virtual void set_driving_agent_class_constant(std::shared_ptr<const IConstant<DrivingAgentClass>> driving_agent_class_constant) = 0;
    virtual void set_position_variable(std::shared_ptr<const IConstant<geometry::Vec>> position_variable) = 0;
    virtual void set_linear_velocity_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_velocity_variable) = 0;
    virtual void set_aligned_linear_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_variable) = 0;
    virtual void set_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec>> linear_acceleration_variable) = 0;
    virtual void set_aligned_linear_acceleration_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_acceleration_variable) = 0;
    virtual void set_external_linear_acceleration_variable(std::shared_ptr<const IConstant<geometry::Vec>> external_linear_acceleration_variable) = 0;
    virtual void set_rotation_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> rotation_variable) = 0;
    virtual void set_steer_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> steer_variable) = 0;
    virtual void set_angular_velocity_variable(std::shared_ptr<const IConstant<FP_DATA_TYPE>> angular_velocity_variable) = 0;
};

}
}
}

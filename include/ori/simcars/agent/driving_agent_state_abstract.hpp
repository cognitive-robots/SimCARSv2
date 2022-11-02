#pragma once

#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingAgentState : public virtual IDrivingAgentState
{
public:
    std::shared_ptr<const IConstant<uint32_t>> get_id_constant() const override;
    std::shared_ptr<const IConstant<bool>> get_ego_constant() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_length_constant() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_width_constant() const override;
    std::shared_ptr<const IConstant<DrivingAgentClass>> get_driving_agent_class_constant() const override;
    std::shared_ptr<const IConstant<geometry::Vec>> get_position_variable() const override;
    std::shared_ptr<const IConstant<geometry::Vec>> get_linear_velocity_variable() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_velocity_variable() const override;
    std::shared_ptr<const IConstant<geometry::Vec>> get_linear_acceleration_variable() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_acceleration_variable() const override;
    std::shared_ptr<const IConstant<geometry::Vec>> get_external_linear_acceleration_variable() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_rotation_variable() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_steer_variable() const override;
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_angular_velocity_variable() const override;
};

}
}
}

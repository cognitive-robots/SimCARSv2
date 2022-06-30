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

class IDrivingSceneState : public virtual IState
{
public:
    virtual std::shared_ptr<const IConstant<uint32_t>> get_id_constant(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<bool>> get_ego_constant(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_length_constant(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_width_constant(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<DrivingAgentClass>> get_road_agent_class_constant(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_position_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_linear_velocity_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_velocity_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<geometry::Vec>> get_linear_acceleration_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_aligned_linear_acceleration_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_rotation_variable(const std::string& driving_agent_name) const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_steer_variable(const std::string& driving_agent_name) const = 0;
};

}
}
}

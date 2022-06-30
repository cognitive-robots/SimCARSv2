#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/entity_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgent : public virtual IEntity
{
public:
    virtual std::shared_ptr<IDrivingAgent> driving_agent_deep_copy() const = 0;

    virtual std::shared_ptr<const IConstant<uint32_t>> get_id_constant() const = 0;
    virtual std::shared_ptr<const IConstant<bool>> get_ego_constant() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_length_constant() const = 0;
    virtual std::shared_ptr<const IConstant<FP_DATA_TYPE>> get_bb_width_constant() const = 0;
    virtual std::shared_ptr<const IConstant<DrivingAgentClass>> get_road_agent_class_constant() const = 0;

    virtual std::shared_ptr<const IVariable<geometry::Vec>> get_position_variable() const = 0;
    virtual std::shared_ptr<const IVariable<geometry::Vec>> get_linear_velocity_variable() const = 0;
    virtual std::shared_ptr<const IVariable<FP_DATA_TYPE>> get_aligned_linear_velocity_variable() const = 0;
    virtual std::shared_ptr<const IVariable<geometry::Vec>> get_linear_acceleration_variable() const = 0;
    virtual std::shared_ptr<const IVariable<FP_DATA_TYPE>> get_aligned_linear_acceleration_variable() const = 0;
    virtual std::shared_ptr<const IVariable<FP_DATA_TYPE>> get_rotation_variable() const = 0;
    virtual std::shared_ptr<const IVariable<FP_DATA_TYPE>> get_steer_variable() const = 0;

    virtual std::shared_ptr<const IDrivingAgentState> get_driving_agent_state(temporal::Time time) const = 0;
};

}
}
}

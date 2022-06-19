
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/event.hpp>
#include <ori/simcars/agent/driving_agent_controller.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

DrivingAgentController::DrivingAgentController(
        std::shared_ptr<const IEntity> entity, std::shared_ptr<const map::IMap<std::string> > map) :
    entity(entity), map(map) {}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> DrivingAgentController::get_indirect_actuation_events(temporal::Time time) const
{
    std::shared_ptr<structures::IStackArray<std::shared_ptr<const IValuelessEvent>>> new_events(
                new structures::stl::STLStackArray<std::shared_ptr<const IValuelessEvent>>());

    try
    {
        std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".aligned_linear_velocity.base");
        std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_goal_value_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".aligned_linear_velocity_goal_value.base");
        std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_goal_duration_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".aligned_linear_velocity_goal_duration.base");

        std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_velocity_variable =
                std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);
        std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_variable =
                std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_goal_value_valueless_variable);
        std::shared_ptr<const IVariable<temporal::Duration>> aligned_linear_velocity_goal_duration_variable =
                std::static_pointer_cast<const IVariable<temporal::Duration>>(aligned_linear_velocity_goal_duration_valueless_variable);

        FP_DATA_TYPE aligned_linear_velocity = aligned_linear_velocity_variable->get_value(time);
        FP_DATA_TYPE aligned_linear_velocity_goal_value = aligned_linear_velocity_goal_value_variable->get_value(time);
        temporal::Duration aligned_linear_velocity_goal_duration = aligned_linear_velocity_goal_duration_variable->get_value(time);

        FP_DATA_TYPE aligned_linear_velocity_error = aligned_linear_velocity_goal_value - aligned_linear_velocity;
        FP_DATA_TYPE aligned_linear_acceleration = aligned_linear_velocity_error / aligned_linear_velocity_goal_duration.count();

        std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_acceleration_event(new Event<FP_DATA_TYPE>(entity->get_name() + ".aligned_linear_acceleration.indirect_actuation", aligned_linear_acceleration, time));
        new_events->push_back(aligned_linear_acceleration_event);
    }
    catch (std::out_of_range)
    {
        // Entity either doesn't have aligned linear velocity variable or goal aligned linear velocity variable
        std::cerr << "Could not actuate aligned linear acceleration variable" << std::endl;
    }

    return new_events;
}

}
}
}

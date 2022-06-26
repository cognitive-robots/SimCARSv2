
#include <ori/simcars/agent/driving_agent_simulator.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

DrivingAgentSimulator::DrivingAgentSimulator(std::shared_ptr<const IScene> scene) : scene(scene) {}

std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> DrivingAgentSimulator::simulate(temporal::Time current_time, temporal::Duration time_step) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    for (size_t i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<const IEntity> entity = (*entities)[i];

        FP_DATA_TYPE aligned_linear_acceleration = 0.0f;
        FP_DATA_TYPE new_aligned_linear_acceleration = 0.0f;
        try
        {
            std::shared_ptr<const IValuelessVariable> aligned_linear_acceleration_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".aligned_linear_acceleration.indirect_actuation");

            std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_acceleration_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_acceleration_valueless_variable);

            try
            {
                aligned_linear_acceleration = aligned_linear_acceleration_variable->get_value(current_time);
                new_aligned_linear_acceleration = aligned_linear_acceleration_variable->get_value(current_time + time_step);
            }
            catch (std::out_of_range)
            {
                // Aligned linear acceleration not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No aligned linear acceleration variable available
            std::cerr << "Entity has no aligned linear acceleration variable" << std::endl;
        }

        FP_DATA_TYPE aligned_linear_velocity = 0.0f;
        try
        {
            std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".aligned_linear_velocity.base");

            std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_velocity_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);

            try
            {
                aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Aligned linear velocity not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No aligned linear velocity variable available
            std::cerr << "Entity has no aligned linear velocity variable" << std::endl;
        }

        FP_DATA_TYPE steer = 0.0f;
        FP_DATA_TYPE new_steer = 0.0f;
        try
        {
            std::shared_ptr<const IValuelessVariable> steer_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".steer.indirect_actuation");

            std::shared_ptr<const IVariable<FP_DATA_TYPE>> steer_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(steer_valueless_variable);

            try
            {
                steer = steer_variable->get_value(current_time);
                new_steer = steer_variable->get_value(current_time + time_step);
            }
            catch (std::out_of_range)
            {
                // Steer not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No steer variable available
            std::cerr << "Entity has no steer variable" << std::endl;
        }

        geometry::Vec linear_acceleration = geometry::Vec::Zero();
        try
        {
            std::shared_ptr<const IValuelessVariable> linear_acceleration_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".linear_acceleration.base");

            std::shared_ptr<const IVariable<geometry::Vec>> linear_acceleration_variable =
                    std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_acceleration_valueless_variable);

            try
            {
                linear_acceleration = linear_acceleration_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Linear acceleration not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No linear acceleration variable available
            std::cerr << "Entity has no linear acceleration variable" << std::endl;
        }

        geometry::Vec linear_velocity = geometry::Vec::Zero();
        try
        {
            std::shared_ptr<const IValuelessVariable> linear_velocity_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".linear_velocity.base");

            std::shared_ptr<const IVariable<geometry::Vec>> linear_velocity_variable =
                    std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_velocity_valueless_variable);

            try
            {
                linear_velocity = linear_velocity_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Linear velocity not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No linear velocity variable available
            std::cerr << "Entity has no linear velocity variable" << std::endl;
        }

        FP_DATA_TYPE angular_velocity = 0.0f;
        try
        {
            std::shared_ptr<const IValuelessVariable> angular_velocity_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".angular_velocity.base");

            std::shared_ptr<const IVariable<FP_DATA_TYPE>> angular_velocity_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(angular_velocity_valueless_variable);

            try
            {
                angular_velocity = angular_velocity_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Angular velocity not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No angular velocity variable available
            std::cerr << "Entity has no angular velocity variable" << std::endl;
        }

        geometry::Vec position = geometry::Vec::Zero();
        try
        {
            std::shared_ptr<const IValuelessVariable> position_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".position.base");

            std::shared_ptr<const IVariable<geometry::Vec>> position_variable =
                    std::static_pointer_cast<const IVariable<geometry::Vec>>(position_valueless_variable);

            try
            {
                position = position_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Position not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No position variable available
            std::cerr << "Entity has no position variable" << std::endl;
        }

        FP_DATA_TYPE rotation = 0.0f;
        try
        {
            std::shared_ptr<const IValuelessVariable> rotation_valueless_variable =
                    entity->get_variable_parameter(entity->get_name() + ".rotation.base");

            std::shared_ptr<const IVariable<FP_DATA_TYPE>> rotation_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(rotation_valueless_variable);

            try
            {
                rotation = rotation_variable->get_value(current_time);
            }
            catch (std::out_of_range)
            {
                // Rotation not available for the current time
            }
        }
        catch (std::out_of_range)
        {
            // No rotation variable available
            std::cerr << "Entity has no rotation variable" << std::endl;
        }

        FP_DATA_TYPE mean_aligned_linear_acceleration = (aligned_linear_acceleration + new_aligned_linear_acceleration) / 2.0f;

        FP_DATA_TYPE new_aligned_linear_velocity = aligned_linear_velocity + mean_aligned_linear_acceleration * time_step.count();

        FP_DATA_TYPE mean_aligned_linear_velocity = (aligned_linear_velocity + new_aligned_linear_velocity) / 2.0f;

        FP_DATA_TYPE mean_steer = (steer + new_steer) / 2.0f;

        FP_DATA_TYPE mean_angular_velocity = mean_steer * mean_aligned_linear_velocity;

        FP_DATA_TYPE new_rotation = rotation + mean_angular_velocity * time_step.count();

        geometry::Vec new_linear_acceleration;
        new_linear_acceleration.x() = new_aligned_linear_acceleration * cos(new_rotation);
        new_linear_acceleration.y() = new_aligned_linear_acceleration * sin(new_rotation);

        geometry::Vec mean_linear_acceleration = (linear_acceleration + new_linear_acceleration) / 2.0f;

        geometry::Vec new_linear_velocity = linear_velocity + mean_linear_acceleration * time_step.count();

        geometry::Vec mean_linear_velocity = (linear_velocity + new_linear_velocity) / 2.0f;

        geometry::Vec new_position = position + mean_linear_velocity * time_step.count();
    }
}

}
}
}

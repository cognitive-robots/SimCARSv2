
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/event.hpp>
#include <ori/simcars/agent/variable.hpp>
#include <ori/simcars/agent/driving_agent_scene.hpp>

#include <tuple>
#include <cmath>
#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

std::ostream& operator <<(std::ostream& output_stream, const temporal::Duration& duration)
{
    return output_stream << std::to_string(duration.count());
}

std::shared_ptr<const DrivingAgentScene> DrivingAgentScene::construct_from(std::shared_ptr<const IScene> scene)
{
    static_assert(GAUSSIAN_KERNEL_SIZE % 2, "Gaussian kernel size must be odd");

    std::shared_ptr<DrivingAgentScene> new_scene(new DrivingAgentScene());

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    FP_DATA_TYPE gaussian_kernel[GAUSSIAN_KERNEL_SIZE];
    FP_DATA_TYPE gaussian_kernel_sum = 0.0f;
    size_t i;
    for (i = 0; i < GAUSSIAN_KERNEL_SIZE; ++i)
    {
        size_t offset = i - (GAUSSIAN_KERNEL_SIZE / 2);
        gaussian_kernel[i] = std::exp(-0.5 * std::pow((offset / GAUSSIAN_STD_DEV), 2.0));
        gaussian_kernel_sum += gaussian_kernel[i];
    }
    for (i = 0; i < GAUSSIAN_KERNEL_SIZE; ++i)
    {
        gaussian_kernel[i] /= gaussian_kernel_sum;
    }

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    for(i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<IEntity> new_entity = (*entities)[i]->deep_copy();

        new_scene->entity_dict.update(new_entity->get_name(), new_entity);

        try
        {
            std::shared_ptr<const IValuelessVariable> lane_valueless_variable =
                    new_entity->get_variable_parameter(new_entity->get_name() + ".lane.base");
        }
        catch (std::out_of_range)
        {
            // Entity doesn't have a lane variable
            std::cerr << "Could not extract lane goal variable" << std::endl;
        }

        try
        {
            std::shared_ptr<const geometry::TrigBuff> trig_buff = geometry::TrigBuff::get_instance();

            std::shared_ptr<const IValuelessVariable> linear_velocity_valueless_variable =
                    new_entity->get_variable_parameter(new_entity->get_name() + ".linear_velocity.base");
            std::shared_ptr<const IValuelessVariable> linear_acceleration_valueless_variable =
                    new_entity->get_variable_parameter(new_entity->get_name() + ".linear_acceleration.indirect_actuation");
            std::shared_ptr<const IValuelessVariable> rotation_valueless_variable =
                    new_entity->get_variable_parameter(new_entity->get_name() + ".rotation.base");

            std::shared_ptr<const IVariable<geometry::Vec>> linear_velocity_variable =
                    std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_velocity_valueless_variable);
            std::shared_ptr<const IVariable<geometry::Vec>> linear_acceleration_variable =
                    std::static_pointer_cast<const IVariable<geometry::Vec>>(linear_acceleration_valueless_variable);
            std::shared_ptr<const IVariable<FP_DATA_TYPE>> rotation_variable =
                    std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(rotation_valueless_variable);

            std::shared_ptr<IVariable<FP_DATA_TYPE>> speed_variable(
                        new Variable<FP_DATA_TYPE>(new_entity->get_name(), "speed", IValuelessVariable::Type::BASE));

            temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
            temporal::Time current_time;
            for (current_time = new_scene->min_temporal_limit; current_time <= new_scene->max_temporal_limit; current_time += time_step)
            {
                try
                {
                    geometry::Vec linear_velocity = linear_velocity_variable->get_value(current_time);
                    FP_DATA_TYPE speed = linear_velocity.norm();
                    std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_event(
                                new Event<FP_DATA_TYPE>(speed_variable->get_full_name(), speed, current_time));
                    speed_variable->add_event(speed_event);
                }
                catch (std::out_of_range)
                {
                    // Attempt to get value from before variable was initialised
                }
            }


            std::shared_ptr<IVariable<FP_DATA_TYPE>> aligned_linear_acceleration_variable(
                        new Variable<FP_DATA_TYPE>(new_entity->get_name(), "aligned_linear_acceleration", IValuelessVariable::Type::INDIRECT_ACTUATION));

            //structures::stl::STLStackArray<std::pair<FP_DATA_TYPE, temporal::Time>> low_pass_accelerations;

            for (current_time = new_scene->min_temporal_limit; current_time <= new_scene->max_temporal_limit; current_time += time_step)
            {
                try
                {
                    geometry::Vec linear_acceleration = linear_acceleration_variable->get_value(current_time);
                    FP_DATA_TYPE rotation = rotation_variable->get_value(current_time);
                    FP_DATA_TYPE aligned_linear_acceleration = (trig_buff->get_rot_mat(-rotation) * linear_acceleration).x();
                    std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_acceleration_event(
                                new Event<FP_DATA_TYPE>(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration, current_time));
                    aligned_linear_acceleration_variable->add_event(aligned_linear_acceleration_event);

                    /*
                    FP_DATA_TYPE low_pass_acceleration = 0.0f;

                    size_t j;
                    for (j = 0; j < GAUSSIAN_KERNEL_SIZE; ++j)
                    {
                        size_t offset = j - (GAUSSIAN_KERNEL_SIZE / 2);
                        temporal::Time adjusted_time =
                                std::min(
                                    std::max(temporal::Time(current_time + offset * time_step), new_scene->min_temporal_limit),
                                    new_scene->max_temporal_limit);
                        geometry::Vec linear_acceleration = linear_acceleration_variable->get_value(adjusted_time);
                        FP_DATA_TYPE rotation = rotation_variable->get_value(current_time);
                        FP_DATA_TYPE aligned_linear_acceleration = (trig_buff->get_rot_mat(-rotation) * linear_acceleration).x();
                        low_pass_acceleration += gaussian_kernel[j] * aligned_linear_acceleration;
                    }

                    low_pass_accelerations.push_back(std::pair<FP_DATA_TYPE, temporal::Time>(low_pass_acceleration, current_time));
                    */
                }
                catch (std::out_of_range)
                {
                    // Attempt to get value from before variable was initialised
                }
            }

            std::shared_ptr<IVariable<FP_DATA_TYPE>> speed_goal_value_variable(
                        new Variable<FP_DATA_TYPE>(
                            new_entity->get_name(), "speed_goal_value", IValuelessVariable::Type::GOAL_VALUE));
            std::shared_ptr<IVariable<temporal::Duration>> speed_goal_duration_variable(
                        new Variable<temporal::Duration>(
                            new_entity->get_name(), "speed_goal_duration", IValuelessVariable::Type::GOAL_DURATION));

            temporal::Duration min_duration_threshold(temporal::DurationRep(1000.0 * MIN_DURATION_THRESHOLD));

            temporal::Time action_start_time = new_scene->min_temporal_limit;
            FP_DATA_TYPE action_start_speed = std::numeric_limits<FP_DATA_TYPE>::max();
            for (current_time = new_scene->min_temporal_limit + time_step; current_time <= new_scene->max_temporal_limit; current_time += time_step)
            {
                try
                {
                    FP_DATA_TYPE current_aligned_linear_acceleration = aligned_linear_acceleration_variable->get_value(current_time);
                    FP_DATA_TYPE previous_aligned_linear_acceleration = aligned_linear_acceleration_variable->get_value(current_time - time_step);

                    if (previous_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            // STATUS QUO

                            if (current_time + time_step > new_scene->max_temporal_limit)
                            {
                                if ((current_time - action_start_time >= min_duration_threshold
                                     && std::abs(speed_variable->get_value(current_time) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                        || action_start_time == new_scene->min_temporal_limit)
                                {
                                    std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                         speed_variable->get_value(current_time),
                                                                                                                         action_start_time));
                                    speed_goal_value_variable->add_event(speed_goal_value_event);

                                    std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                         current_time - action_start_time,
                                                                                                                         action_start_time));
                                    speed_goal_duration_variable->add_event(speed_goal_duration_event);
                                }
                            }
                        }
                        else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            if (((current_time - time_step) - action_start_time >= min_duration_threshold
                                 && std::abs(speed_variable->get_value(current_time - time_step) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                    || (current_time + time_step > new_scene->max_temporal_limit
                                        && action_start_time == new_scene->min_temporal_limit))
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time - time_step),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }

                            action_start_time = current_time - time_step;
                            action_start_speed = speed_variable->get_value(current_time - time_step);
                        }
                        else
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(speed_variable->get_value(current_time) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                    || (current_time + time_step > new_scene->max_temporal_limit
                                        && action_start_time == new_scene->min_temporal_limit))
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }
                        }
                    }
                    else if (previous_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            if (((current_time - time_step) - action_start_time >= min_duration_threshold
                                 && std::abs(speed_variable->get_value(current_time - time_step) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                    || (current_time + time_step > new_scene->max_temporal_limit
                                        && action_start_time == new_scene->min_temporal_limit))
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time - time_step),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }

                            action_start_time = current_time - time_step;
                            action_start_speed = speed_variable->get_value(current_time - time_step);
                        }
                        else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            // STATUS QUO

                            if (current_time + time_step > new_scene->max_temporal_limit)
                            {
                                if ((current_time - action_start_time >= min_duration_threshold
                                     && std::abs(speed_variable->get_value(current_time) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                        || action_start_time == new_scene->min_temporal_limit)
                                {
                                    std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                         speed_variable->get_value(current_time),
                                                                                                                         action_start_time));
                                    speed_goal_value_variable->add_event(speed_goal_value_event);

                                    std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                         current_time - action_start_time,
                                                                                                                         action_start_time));
                                    speed_goal_duration_variable->add_event(speed_goal_duration_event);
                                }
                            }
                        }
                        else
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(speed_variable->get_value(current_time) - action_start_speed) >= MIN_SPEED_DIFF_THRESHOLD)
                                    || (current_time + time_step > new_scene->max_temporal_limit
                                        && action_start_time == new_scene->min_temporal_limit))
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }
                        }
                    }
                    else
                    {
                        if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            if (action_start_time == new_scene->min_temporal_limit)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time - time_step),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }

                            action_start_time = current_time - time_step;
                            action_start_speed = speed_variable->get_value(current_time - time_step);
                        }
                        else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            if (action_start_time == new_scene->min_temporal_limit)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time - time_step),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }

                            action_start_time = current_time - time_step;
                            action_start_speed = speed_variable->get_value(current_time - time_step);
                        }
                        else
                        {
                            // STATUS QUO

                            if (current_time + time_step > new_scene->max_temporal_limit && action_start_time == new_scene->min_temporal_limit)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(current_time),
                                                                                                                     action_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - action_start_time,
                                                                                                                     action_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }
                        }
                    }
                }
                catch (std::out_of_range)
                {
                    // Attempt to get value from before variable was initialised
                }
            }

            /*
            if (low_pass_accelerations.count() > 0)
            {
                temporal::Duration min_duration_threshold(
                            temporal::DurationRep(1000.0 * min_duration_threshold));

                size_t current_cluster_start = 0;
                size_t current_cluster_initial_count = 1;
                FP_DATA_TYPE current_cluster_initial_sum = low_pass_accelerations[0].first;
                temporal::Time current_cluster_start_time = low_pass_accelerations[0].second;

                size_t j;
                temporal::Time previous_time = low_pass_accelerations[0].second;
                for (j = 1; j < low_pass_accelerations.count(); ++j)
                {
                    std::pair<FP_DATA_TYPE, temporal::Time> low_pass_accleration_time_pair = low_pass_accelerations[j];
                    FP_DATA_TYPE low_pass_acceleration = low_pass_accleration_time_pair.first;
                    temporal::Time time = low_pass_accleration_time_pair.second;

                    if (previous_time - current_cluster_start_time >= min_duration_threshold)
                    {
                        FP_DATA_TYPE current_cluster_mean = (current_cluster_initial_sum / current_cluster_initial_count);
                        if (std::abs(low_pass_acceleration - current_cluster_mean) > ACCELERATION_CLUSTERING_THRESHOLD)
                        {
                            if (std::abs(current_cluster_mean) >= ACTION_BACKED_ACCELERATION_THRESHOLD
                                    || current_cluster_start == 0)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                     speed_variable->get_value(previous_time),
                                                                                                                     current_cluster_start_time));
                                speed_goal_value_variable->add_event(speed_goal_value_event);

                                std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                     previous_time - current_cluster_start_time,
                                                                                                                     current_cluster_start_time));
                                speed_goal_duration_variable->add_event(speed_goal_duration_event);
                            }

                            current_cluster_start = j;
                            current_cluster_initial_count = 0;
                            current_cluster_initial_sum = 0.0f;
                            current_cluster_start_time = previous_time;
                        }
                    }
                    else
                    {
                        ++current_cluster_initial_count;
                        current_cluster_initial_sum += low_pass_acceleration;
                    }

                    previous_time = time;
                }

                if (current_cluster_start == 0)
                {
                    std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                         speed_variable->get_value(previous_time),
                                                                                                         current_cluster_start_time));
                    speed_goal_value_variable->add_event(speed_goal_value_event);

                    std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                         previous_time - current_cluster_start_time,
                                                                                                         current_cluster_start_time));
                    speed_goal_duration_variable->add_event(speed_goal_duration_event);
                }
                else
                {
                    if (previous_time - current_cluster_start_time >= min_duration_threshold)
                    {
                        FP_DATA_TYPE current_cluster_mean = (current_cluster_initial_sum / current_cluster_initial_count);
                        if (std::abs(current_cluster_mean) >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> speed_goal_value_event(new Event<FP_DATA_TYPE>(speed_goal_value_variable->get_full_name(),
                                                                                                                 speed_variable->get_value(previous_time),
                                                                                                                 current_cluster_start_time));
                            speed_goal_value_variable->add_event(speed_goal_value_event);

                            std::shared_ptr<IEvent<temporal::Duration>> speed_goal_duration_event(new Event<temporal::Duration>(speed_goal_duration_variable->get_full_name(),
                                                                                                                 previous_time - current_cluster_start_time,
                                                                                                                 current_cluster_start_time));
                            speed_goal_duration_variable->add_event(speed_goal_duration_event);
                        }
                    }
                }
            }
            */

            new_entity->add_variable_parameter(speed_variable->get_full_name(), speed_variable);
            new_entity->add_variable_parameter(aligned_linear_acceleration_variable->get_full_name(), aligned_linear_acceleration_variable);
            new_entity->add_variable_parameter(speed_goal_value_variable->get_full_name(), speed_goal_value_variable);
            new_entity->add_variable_parameter(speed_goal_duration_variable->get_full_name(), speed_goal_duration_variable);
        }
        catch (std::out_of_range)
        {
            // Entity doesn't have a linear velocity variable
            std::cerr << "Could not extract speed variable and speed goal variable" << std::endl;
        }
    }

    return new_scene;
}

geometry::Vec DrivingAgentScene::get_min_spatial_limits() const
{
    return min_spatial_limits;
}

geometry::Vec DrivingAgentScene::get_max_spatial_limits() const
{
    return max_spatial_limits;
}

temporal::Time DrivingAgentScene::get_min_temporal_limit() const
{
    return min_temporal_limit;
}

temporal::Time DrivingAgentScene::get_max_temporal_limit() const
{
    return max_temporal_limit;
}

std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> DrivingAgentScene::get_entities() const
{
    return entity_dict.get_values();
}

std::shared_ptr<const IEntity> DrivingAgentScene::get_entity(const std::string& entity_name) const
{
    return entity_dict[entity_name];
}

}
}
}

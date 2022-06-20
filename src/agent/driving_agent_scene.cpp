
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/map/lane_interface.hpp>
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

void DrivingAgentScene::extract_aligned_linear_velocity_change_events(std::shared_ptr<IEntity> entity)
{
    try
    {
        std::shared_ptr<const IValuelessVariable> aligned_linear_velocity_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".aligned_linear_velocity.base");
        std::shared_ptr<const IValuelessVariable> aligned_linear_acceleration_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".aligned_linear_acceleration.indirect_actuation");

        std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_velocity_variable =
                std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_velocity_valueless_variable);
        std::shared_ptr<const IVariable<FP_DATA_TYPE>> aligned_linear_acceleration_variable =
                std::static_pointer_cast<const IVariable<FP_DATA_TYPE>>(aligned_linear_acceleration_valueless_variable);

        std::shared_ptr<IVariable<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_variable(
                    new Variable<FP_DATA_TYPE>(
                        entity->get_name(), "aligned_linear_velocity_goal_value", IValuelessVariable::Type::GOAL_VALUE));
        std::shared_ptr<IVariable<temporal::Duration>> aligned_linear_velocity_goal_duration_variable(
                    new Variable<temporal::Duration>(
                        entity->get_name(), "aligned_linear_velocity_goal_duration", IValuelessVariable::Type::GOAL_DURATION));

        temporal::Duration min_duration_threshold(temporal::DurationRep(1000.0 * MIN_ALIGNED_LINEAR_VELOCITY_CHANGE_DURATION_THRESHOLD));

        temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
        temporal::Time current_time;
        temporal::Time action_start_time = this->min_temporal_limit;
        FP_DATA_TYPE action_start_aligned_linear_velocity = std::numeric_limits<FP_DATA_TYPE>::max();
        for (current_time = this->min_temporal_limit + time_step; current_time <= this->max_temporal_limit; current_time += time_step)
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

                        if (current_time + time_step > this->max_temporal_limit)
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                    || action_start_time == this->min_temporal_limit)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                     aligned_linear_velocity_variable->get_value(current_time),
                                                                                                                     action_start_time));
                                aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                                for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                                {
                                    std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                         current_time - current_time_2,
                                                                                                                         current_time_2));
                                    aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                                }
                            }
                        }
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (((current_time - time_step) - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time - time_step) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->max_temporal_limit
                                    && action_start_time == this->min_temporal_limit))
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time - time_step),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else
                    {
                        if ((current_time - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->max_temporal_limit
                                    && action_start_time == this->min_temporal_limit))
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }
                    }
                }
                else if (previous_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                {
                    if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (((current_time - time_step) - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time - time_step) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->max_temporal_limit
                                    && action_start_time == this->min_temporal_limit))
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time - time_step),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        // STATUS QUO

                        if (current_time + time_step > this->max_temporal_limit)
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                    || action_start_time == this->min_temporal_limit)
                            {
                                std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                     aligned_linear_velocity_variable->get_value(current_time),
                                                                                                                     action_start_time));
                                aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                                for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                                {
                                    std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                         current_time - current_time_2,
                                                                                                                         current_time_2));
                                    aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                                }
                            }
                        }
                    }
                    else
                    {
                        if ((current_time - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->max_temporal_limit
                                    && action_start_time == this->min_temporal_limit))
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }
                    }
                }
                else
                {
                    if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (action_start_time == this->min_temporal_limit)
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time - time_step),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (action_start_time == this->min_temporal_limit)
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time - time_step),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     (current_time - time_step) - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else
                    {
                        // STATUS QUO

                        if (current_time + time_step > this->max_temporal_limit && action_start_time == this->min_temporal_limit)
                        {
                            std::shared_ptr<IEvent<FP_DATA_TYPE>> aligned_linear_velocity_goal_value_event(new Event<FP_DATA_TYPE>(aligned_linear_velocity_goal_value_variable->get_full_name(),
                                                                                                                 aligned_linear_velocity_variable->get_value(current_time),
                                                                                                                 action_start_time));
                            aligned_linear_velocity_goal_value_variable->add_event(aligned_linear_velocity_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> aligned_linear_velocity_goal_duration_event(new Event<temporal::Duration>(aligned_linear_velocity_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - current_time_2,
                                                                                                                     current_time_2));
                                aligned_linear_velocity_goal_duration_variable->add_event(aligned_linear_velocity_goal_duration_event);
                            }
                        }
                    }
                }
            }
            catch (std::out_of_range)
            {
                // Attempt to get value from before variable was initialised
            }
        }

        entity->add_variable_parameter(aligned_linear_velocity_goal_value_variable->get_full_name(), aligned_linear_velocity_goal_value_variable);
        entity->add_variable_parameter(aligned_linear_velocity_goal_duration_variable->get_full_name(), aligned_linear_velocity_goal_duration_variable);
    }
    catch (std::out_of_range)
    {
        // Entity doesn't have a linear velocity variable
        std::cerr << "Could not extract aligned linear velocity goal variables" << std::endl;
    }
}

void DrivingAgentScene::extract_lane_change_events(std::shared_ptr<IEntity> entity, std::shared_ptr<const map::IMap<std::string> > map)
{
    try
    {
        std::shared_ptr<const IValuelessVariable> position_valueless_variable =
                entity->get_variable_parameter(entity->get_name() + ".position.base");

        std::shared_ptr<const IVariable<geometry::Vec>> position_variable =
                std::static_pointer_cast<const IVariable<geometry::Vec>>(position_valueless_variable);

        temporal::Duration min_duration_threshold(temporal::DurationRep(1000.0 * MIN_LANE_CHANGE_DURATION_THRESHOLD));

        std::shared_ptr<IVariable<int32_t>> lane_goal_value_variable(
                    new Variable<int32_t>(
                        entity->get_name(), "lane_goal_value", IValuelessVariable::Type::GOAL_VALUE));
        std::shared_ptr<IVariable<temporal::Duration>> lane_goal_duration_variable(
                    new Variable<temporal::Duration>(
                        entity->get_name(), "lane_goal_duration", IValuelessVariable::Type::GOAL_DURATION));

        temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
        temporal::Time current_time;
        temporal::Time action_start_time;
        temporal::Time lane_change_time = temporal::Time::max();
        int32_t lane_change_offset = 0;
        std::shared_ptr<const map::ILaneArray<std::string>> previous_lanes = nullptr;
        for (current_time = this->min_temporal_limit; current_time <= this->max_temporal_limit; current_time += time_step)
        {
            try
            {
                geometry::Vec current_position = position_variable->get_value(current_time);
                std::shared_ptr<const map::ILaneArray<std::string>> current_lanes =
                        map->get_encapsulating_lanes(current_position);

                if (current_lanes->count() == 0)
                {
                    // Vehicle is currently not on a lane segment. This can happen quite often due to small gaps inbetween the lane segments. As such we just disregard this timestep.
                    continue;
                }

                if (previous_lanes != nullptr)
                {
                    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>> continuing_lanes(
                                new structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>());
                    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>> left_lanes(
                                new structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>());
                    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>> right_lanes(
                                new structures::stl::STLStackArray<std::shared_ptr<const map::ILane<std::string>>>());
                    for (size_t j = 0; j < current_lanes->count(); ++j)
                    {
                        std::shared_ptr<const map::ILane<std::string>> current_lane = (*current_lanes)[j];
                        for (size_t k = 0; k < previous_lanes->count(); ++k)
                        {
                            std::shared_ptr<const map::ILane<std::string>> previous_lane = (*previous_lanes)[k];
                            if (current_lane == previous_lane || current_lane->get_aft_lanes()->contains(previous_lane))
                            {
                                continuing_lanes->push_back(current_lane);
                            }
                            else if (current_lane->get_right_adjacent_lane() == previous_lane)
                            {
                                left_lanes->push_back(current_lane);
                            }
                            else if (current_lane->get_left_adjacent_lane() == previous_lane)
                            {
                                right_lanes->push_back(current_lane);
                            }
                        }
                    }

                    if (continuing_lanes->count() == 0)
                    {
                        if (left_lanes->count() > 0)
                        {
                            --lane_change_offset;
                            lane_change_time = current_time;
                            previous_lanes = left_lanes;
                        }
                        else if (right_lanes->count() > 0)
                        {
                            ++lane_change_offset;
                            lane_change_time = current_time;
                            previous_lanes = right_lanes;
                        }
                        else
                        {
                            // This should be an exceptionally rare occurance, the vehicle is occupying at least one valid lane, but none of the occupied lanes are adjacent to the previously occupied lanes.
                            // Because this is so rare, and difficult to handle, for now we essentially treat it as though the current lane occupation is a continuation of the previous lane occupation.
                            previous_lanes = current_lanes;
                        }
                    }
                    else
                    {
                        // We've found a valid continuation for the previous lanes, so now update the previous lanes with the lanes found to be valid continuations.
                        previous_lanes = continuing_lanes;
                    }
                }
                else
                {
                    // There were no previous lanes from which to calculate continuing lanes, so use current lanes for updating the previous lanes.
                    previous_lanes = current_lanes;
                }

                if (lane_change_time != temporal::Time::max())
                {
                    if (current_time - lane_change_time >= min_duration_threshold)
                    {
                        if (lane_change_offset != 0)
                        {
                            std::shared_ptr<IEvent<int32_t>> lane_goal_value_event(new Event<int32_t>(lane_goal_value_variable->get_full_name(),
                                                                                                                 lane_change_offset,
                                                                                                                 action_start_time));
                            lane_goal_value_variable->add_event(lane_goal_value_event);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                std::shared_ptr<IEvent<temporal::Duration>> lane_goal_duration_event(new Event<temporal::Duration>(lane_goal_duration_variable->get_full_name(),
                                                                                                                     current_time - current_time_2,
                                                                                                                     current_time_2));
                                lane_goal_duration_variable->add_event(lane_goal_duration_event);
                            }
                        }
                        else
                        {
                            // We ended up in the same lane as we started with too little time in any other lane to consider a lane change to have occured.
                            // In other words, do nothing.
                        }

                        lane_change_time = temporal::Time::max();
                    }
                    else
                    {
                        // We haven't been in this lane long enough to consider it an actual lane change. Keep waiting!
                    }
                }
                else
                {
                    action_start_time = current_time;
                }
            }
            catch (std::out_of_range)
            {
                // Attempt to get value from before variable was initialised
            }
        }

        entity->add_variable_parameter(lane_goal_value_variable->get_full_name(), lane_goal_value_variable);
        entity->add_variable_parameter(lane_goal_duration_variable->get_full_name(), lane_goal_duration_variable);
    }
    catch (std::out_of_range)
    {
        // Entity doesn't have a lane variable
        std::cerr << "Could not extract lane goal variable" << std::endl;
    }
}

std::shared_ptr<const DrivingAgentScene> DrivingAgentScene::construct_from(std::shared_ptr<const IScene> scene)
{
    std::shared_ptr<DrivingAgentScene> new_scene(new DrivingAgentScene());

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    for(i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<IEntity> new_entity = (*entities)[i]->deep_copy();

        new_scene->entity_dict.update(new_entity->get_name(), new_entity);

        new_scene->extract_aligned_linear_velocity_change_events(new_entity);
    }

    return new_scene;
}

std::shared_ptr<const DrivingAgentScene> DrivingAgentScene::construct_from(std::shared_ptr<const IScene> scene,
                                                                           std::shared_ptr<const map::IMap<std::string>> map)
{
    std::shared_ptr<DrivingAgentScene> new_scene(new DrivingAgentScene());
    new_scene->map = map;

    new_scene->min_spatial_limits = scene->get_min_spatial_limits();
    new_scene->max_spatial_limits = scene->get_max_spatial_limits();
    new_scene->min_temporal_limit = scene->get_min_temporal_limit();
    new_scene->max_temporal_limit = scene->get_max_temporal_limit();

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> entities = scene->get_entities();

    for(i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<IEntity> new_entity = (*entities)[i]->deep_copy();

        new_scene->entity_dict.update(new_entity->get_name(), new_entity);

        new_scene->extract_lane_change_events(new_entity, map);

        new_scene->extract_aligned_linear_velocity_change_events(new_entity);
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
    return std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>>(
            new structures::stl::STLStackArray<std::shared_ptr<const IEntity>>(entity_dict.get_values()));
}

std::shared_ptr<const IEntity> DrivingAgentScene::get_entity(const std::string& entity_name) const
{
    return entity_dict[entity_name];
}

bool DrivingAgentScene::has_map() const
{
    return map != nullptr;
}

std::shared_ptr<const map::IMap<std::string>> DrivingAgentScene::get_map() const
{
    return map;
}

}
}
}

#pragma once

#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/map/living_lane_stack_array.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/agent/basic_variable.hpp>
#include <ori/simcars/agent/basic_goal_driving_agent_state.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T_map_id>
class DrivingGoalExtractionAgent : public virtual ADrivingAgent
{
    IDrivingAgent *driving_agent;

    structures::stl::STLDictionary<std::string, IValuelessVariable*> variable_dict;

    DrivingGoalExtractionAgent()
    {
    }

    void extract_aligned_linear_velocity_change_events()
    {
        IVariable<FP_DATA_TYPE> const *aligned_linear_velocity_variable =
                this->get_aligned_linear_velocity_variable();
        IVariable<FP_DATA_TYPE> const *aligned_linear_acceleration_variable =
                this->get_aligned_linear_acceleration_variable();

        IVariable<FP_DATA_TYPE> *aligned_linear_velocity_goal_value_variable =
                    new BasicVariable<FP_DATA_TYPE>(
                        this->get_name(), "aligned_linear_velocity", IValuelessVariable::Type::GOAL_VALUE);
        IVariable<temporal::Duration> *aligned_linear_velocity_goal_duration_variable =
                    new BasicVariable<temporal::Duration>(
                        this->get_name(), "aligned_linear_velocity", IValuelessVariable::Type::GOAL_DURATION);

        temporal::Duration min_duration_threshold(temporal::DurationRep(1000.0 * MIN_ALIGNED_LINEAR_VELOCITY_CHANGE_DURATION_THRESHOLD));

        temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
        temporal::Time current_time;
        temporal::Time action_start_time = this->get_min_temporal_limit();
        FP_DATA_TYPE action_start_aligned_linear_velocity = std::numeric_limits<FP_DATA_TYPE>::max();
        for (current_time = this->get_min_temporal_limit() + time_step; current_time <= this->get_max_temporal_limit(); current_time += time_step)
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

                        if (current_time + time_step > this->get_max_temporal_limit())
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                    || action_start_time == this->get_min_temporal_limit())
                            {
                                aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time));

                                for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                                {
                                    aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
                                }
                            }
                        }
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (((current_time - time_step) - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time - time_step) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->get_max_temporal_limit()
                                    && action_start_time == this->get_min_temporal_limit()))
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time - time_step));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, (current_time - time_step) - current_time_2);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else
                    {
                        if ((current_time - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->get_max_temporal_limit()
                                    && action_start_time == this->get_min_temporal_limit()))
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
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
                                || (current_time + time_step > this->get_max_temporal_limit()
                                    && action_start_time == this->get_min_temporal_limit()))
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time - time_step));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, (current_time - time_step) - current_time_2);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        // STATUS QUO

                        if (current_time + time_step > this->get_max_temporal_limit())
                        {
                            if ((current_time - action_start_time >= min_duration_threshold
                                 && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                    || action_start_time == this->get_min_temporal_limit())
                            {
                                aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time));

                                for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                                {
                                    aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
                                }
                            }
                        }
                    }
                    else
                    {
                        if ((current_time - action_start_time >= min_duration_threshold
                             && std::abs(aligned_linear_velocity_variable->get_value(current_time) - action_start_aligned_linear_velocity) >= MIN_ALIGNED_LINEAR_VELOCITY_DIFF_THRESHOLD)
                                || (current_time + time_step > this->get_max_temporal_limit()
                                    && action_start_time == this->get_min_temporal_limit()))
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
                            }
                        }
                    }
                }
                else
                {
                    if (current_aligned_linear_acceleration >= ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (action_start_time == this->get_min_temporal_limit())
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time - time_step));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, (current_time - time_step) - current_time_2);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else if (current_aligned_linear_acceleration <= -ACTION_BACKED_ACCELERATION_THRESHOLD)
                    {
                        if (action_start_time == this->get_min_temporal_limit())
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time - time_step));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time - time_step; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, (current_time - time_step) - current_time_2);
                            }
                        }

                        action_start_time = current_time - time_step;
                        action_start_aligned_linear_velocity = aligned_linear_velocity_variable->get_value(current_time - time_step);
                    }
                    else
                    {
                        // STATUS QUO

                        if (current_time + time_step > this->get_max_temporal_limit() && action_start_time == this->get_min_temporal_limit())
                        {
                            aligned_linear_velocity_goal_value_variable->set_value(action_start_time, aligned_linear_velocity_variable->get_value(current_time));

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                aligned_linear_velocity_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
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

        structures::IArray<IEvent<FP_DATA_TYPE> const*> *events = aligned_linear_velocity_goal_value_variable->get_events();
        if (events->count() == 0 ||
                aligned_linear_velocity_goal_value_variable->get_min_temporal_limit() > this->get_min_temporal_limit())
        {
            aligned_linear_velocity_goal_value_variable->set_value(this->get_min_temporal_limit(), aligned_linear_velocity_variable->get_value(this->get_min_temporal_limit()));
            aligned_linear_velocity_goal_duration_variable->set_value(this->get_min_temporal_limit(), temporal::Duration(0));
        }
        delete events;

        this->variable_dict.update(aligned_linear_velocity_goal_value_variable->get_full_name(), aligned_linear_velocity_goal_value_variable);
        this->variable_dict.update(aligned_linear_velocity_goal_duration_variable->get_full_name(), aligned_linear_velocity_goal_duration_variable);
    }
    void extract_lane_change_events(map::IMap<T_map_id> const *map)
    {
        IVariable<geometry::Vec> const *position_variable = this->get_position_variable();

        temporal::Duration min_duration_threshold(temporal::DurationRep(1000.0 * MIN_LANE_CHANGE_DURATION_THRESHOLD));

        IVariable<int32_t> *lane_goal_value_variable =
                    new BasicVariable<int32_t>(
                        this->get_name(), "lane", IValuelessVariable::Type::GOAL_VALUE);
        IVariable<temporal::Duration> *lane_goal_duration_variable =
                    new BasicVariable<temporal::Duration>(
                        this->get_name(), "lane", IValuelessVariable::Type::GOAL_DURATION);

        temporal::Duration time_step = temporal::Duration(temporal::DurationRep(1000.0 / OPERATING_FRAMERATE));
        temporal::Time current_time;
        temporal::Time action_start_time;
        temporal::Time lane_change_time = temporal::Time::max();
        int32_t lane_change_offset = 0;
        map::ILaneArray<T_map_id> const *previous_lanes = nullptr;
        for (current_time = this->get_min_temporal_limit(); current_time <= this->get_max_temporal_limit(); current_time += time_step)
        {
            map::ILaneArray<T_map_id> const *current_lanes = nullptr;
            map::LivingLaneStackArray<T_map_id> *continuing_lanes = nullptr;
            map::LivingLaneStackArray<T_map_id> *left_lanes = nullptr;
            map::LivingLaneStackArray<T_map_id> *right_lanes = nullptr;

            try
            {
                geometry::Vec current_position = position_variable->get_value(current_time);
                current_lanes = map->get_encapsulating_lanes(current_position);

                if (current_lanes->count() == 0)
                {
                    // Vehicle is currently not on a lane segment. This can happen quite often due to small gaps inbetween the lane segments. As such we just disregard this timestep.
                    delete current_lanes;
                    continue;
                }

                if (previous_lanes != nullptr)
                {
                    continuing_lanes = new map::LivingLaneStackArray<T_map_id>;
                    left_lanes = new map::LivingLaneStackArray<T_map_id>;
                    right_lanes = new map::LivingLaneStackArray<T_map_id>;
                    for (size_t j = 0; j < current_lanes->count(); ++j)
                    {
                        map::ILane<T_map_id> const *current_lane = (*current_lanes)[j];
                        map::ILaneArray<T_map_id> const *current_lane_aft_lanes = current_lane->get_aft_lanes();
                        for (size_t k = 0; k < previous_lanes->count(); ++k)
                        {
                            map::ILane<T_map_id> const *previous_lane = (*previous_lanes)[k];
                            if (current_lane == previous_lane || (current_lane_aft_lanes != nullptr && current_lane_aft_lanes->contains(previous_lane)))
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
                            delete previous_lanes;
                            previous_lanes = left_lanes;
                            left_lanes = nullptr;
                        }
                        else if (right_lanes->count() > 0)
                        {
                            ++lane_change_offset;
                            lane_change_time = current_time;
                            delete previous_lanes;
                            previous_lanes = right_lanes;
                            right_lanes = nullptr;
                        }
                        else
                        {
                            // This should be an exceptionally rare occurance, the vehicle is occupying at least one valid lane, but none of the occupied lanes are adjacent to the previously occupied lanes.
                            // Because this is so rare, and difficult to handle, for now we essentially treat it as though the current lane occupation is a continuation of the previous lane occupation.
                            delete previous_lanes;
                            previous_lanes = current_lanes;
                            current_lanes = nullptr;
                        }
                    }
                    else
                    {
                        // We've found a valid continuation for the previous lanes, so now update the previous lanes with the lanes found to be valid continuations.
                        delete previous_lanes;
                        previous_lanes = continuing_lanes;
                        continuing_lanes = nullptr;
                    }
                }
                else
                {
                    // There were no previous lanes from which to calculate continuing lanes, so use current lanes for updating the previous lanes.
                    previous_lanes = current_lanes;
                    current_lanes = nullptr;
                }

                if (lane_change_time != temporal::Time::max())
                {
                    if (current_time - lane_change_time >= min_duration_threshold)
                    {
                        if (lane_change_offset != 0)
                        {
                            lane_goal_value_variable->set_value(action_start_time, lane_change_offset);

                            for (temporal::Time current_time_2 = action_start_time; current_time_2 <= current_time; current_time_2 += time_step)
                            {
                                lane_goal_duration_variable->set_value(current_time_2, current_time - current_time_2);
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

            delete current_lanes;
            delete continuing_lanes;
            delete left_lanes;
            delete right_lanes;
        }

        delete previous_lanes;

        this->variable_dict.update(lane_goal_value_variable->get_full_name(), lane_goal_value_variable);
        this->variable_dict.update(lane_goal_duration_variable->get_full_name(), lane_goal_duration_variable);
    }

public:
    DrivingGoalExtractionAgent(IDrivingAgent *driving_agent)
        : driving_agent(driving_agent)
    {
        structures::IArray<IValuelessVariable*> const *variables =
                driving_agent->get_mutable_variable_parameters();

        for(size_t i = 0; i < variables->count(); ++i)
        {
            this->variable_dict.update((*variables)[i]->get_full_name(), (*variables)[i]);
        }

        delete variables;

        this->extract_aligned_linear_velocity_change_events();
    }
    DrivingGoalExtractionAgent(IDrivingAgent *driving_agent, map::IMap<T_map_id> const *map)
        : DrivingGoalExtractionAgent(driving_agent)
    {
        this->extract_lane_change_events(map);
    }

    ~DrivingGoalExtractionAgent()
    {
        std::string variable_name;

        variable_name = this->get_name() + ".aligned_linear_velocity.goal_value";
        if (variable_dict.contains(variable_name))
        {
            delete variable_dict[variable_name];
        }

        variable_name = this->get_name() + ".aligned_linear_velocity.goal_duration";
        if (variable_dict.contains(variable_name))
        {
            delete variable_dict[variable_name];
        }

        variable_name = this->get_name() + ".lane.goal_value";
        if (variable_dict.contains(variable_name))
        {
            delete variable_dict[variable_name];
        }

        variable_name = this->get_name() + ".lane.goal_duration";
        if (variable_dict.contains(variable_name))
        {
            delete variable_dict[variable_name];
        }
    }

    std::string get_name() const override
    {
        return this->driving_agent->get_name();
    }

    geometry::Vec get_min_spatial_limits() const override
    {
        return this->driving_agent->get_min_spatial_limits();
    }
    geometry::Vec get_max_spatial_limits() const override
    {
        return this->driving_agent->get_max_spatial_limits();
    }

    temporal::Time get_min_temporal_limit() const override
    {
        return this->driving_agent->get_min_temporal_limit();
    }
    temporal::Time get_max_temporal_limit() const override
    {
        return this->driving_agent->get_max_temporal_limit();
    }

    structures::IArray<IValuelessConstant const*>* get_constant_parameters() const override
    {
        return this->driving_agent->get_constant_parameters();
    }
    IValuelessConstant const* get_constant_parameter(std::string const &constant_name) const override
    {
        return this->driving_agent->get_constant_parameter(constant_name);
    }

    structures::IArray<IValuelessVariable const*>* get_variable_parameters() const override
    {
        structures::stl::STLStackArray<IValuelessVariable const*> *variables =
                new structures::stl::STLStackArray<IValuelessVariable const*>(variable_dict.count());
        cast_array(*variable_dict.get_values(), *variables);
        return variables;
    }
    IValuelessVariable const* get_variable_parameter(std::string const &variable_name) const override
    {
        return variable_dict[variable_name];
    }

    structures::IArray<IValuelessEvent const*>* get_events() const override
    {
        structures::IArray<std::string> const *variable_names = variable_dict.get_keys();

        structures::stl::STLConcatArray<IValuelessEvent const*> *events =
                    new structures::stl::STLConcatArray<IValuelessEvent const*>(variable_names->count());

        size_t i;
        for(i = 0; i < variable_names->count(); ++i)
        {
            events->get_array(i) = variable_dict[(*variable_names)[i]]->get_valueless_events();
        }

        return events;
    }

    IDrivingAgent* driving_agent_deep_copy() const override
    {
        DrivingGoalExtractionAgent *driving_agent = new DrivingGoalExtractionAgent;

        driving_agent->driving_agent = this->driving_agent;

        structures::IArray<std::string> const *variable_names = this->variable_dict.get_keys();
        for(size_t i = 0; i < variable_names->count(); ++i)
        {
            driving_agent->variable_dict.update((*variable_names)[i], variable_dict[(*variable_names)[i]]->valueless_deep_copy());
        }

        return driving_agent;
    }


    structures::IArray<IValuelessConstant*>* get_mutable_constant_parameters() override
    {
        return driving_agent->get_mutable_constant_parameters();
    }
    IValuelessConstant* get_mutable_constant_parameter(std::string const &constant_name) override
    {
        return driving_agent->get_mutable_constant_parameter(constant_name);
    }

    structures::IArray<IValuelessVariable*>* get_mutable_variable_parameters() override
    {
        structures::stl::STLStackArray<IValuelessVariable*> *variables =
                new structures::stl::STLStackArray<IValuelessVariable*>(variable_dict.count());
        variable_dict.get_values(variables);
        return variables;
    }
    IValuelessVariable* get_mutable_variable_parameter(std::string const &variable_name) override
    {
        return variable_dict[variable_name];
    }

    structures::IArray<IValuelessEvent*>* get_mutable_events() override
    {
        structures::IArray<std::string> const *variable_names = variable_dict.get_keys();

        structures::stl::STLConcatArray<IValuelessEvent*> *events =
                new structures::stl::STLConcatArray<IValuelessEvent*>(variable_names->count());

        size_t i;
        for(i = 0; i < variable_names->count(); ++i)
        {
            events->get_array(i) = variable_dict[(*variable_names)[i]]->get_mutable_valueless_events();
        }

        return events;
    }
};

}
}
}

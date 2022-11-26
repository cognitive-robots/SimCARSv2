#pragma once

#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/map/lane_array_interface.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/driving_agent_controller_interface.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T_map_id>
class BasicDrivingAgentController : public virtual IDrivingAgentController
{
    geometry::TrigBuff const *trig_buff;

    map::IMap<T_map_id> const *map;

    temporal::Duration time_step;
    size_t steering_lookahead_steps;

public:
    BasicDrivingAgentController(map::IMap<T_map_id> const *map, temporal::Duration time_step, size_t steering_lookahead_steps)
        : trig_buff(geometry::TrigBuff::get_instance()), map(map), time_step(time_step),
          steering_lookahead_steps(steering_lookahead_steps)
    {
    }

    void modify_state(agent::IState const *original_state, agent::IState *modified_state) const override
    {
        this->modify_driving_agent_state(
                    dynamic_cast<IDrivingAgentState const*>(original_state),
                    dynamic_cast<IDrivingAgentState*>(modified_state));
    }

    void modify_driving_agent_state(agent::IDrivingAgentState const *original_state,
                                    agent::IDrivingAgentState *modified_state) const override
    {
        FP_DATA_TYPE aligned_linear_velocity = original_state->get_aligned_linear_velocity_variable()->get_value();

        FP_DATA_TYPE new_aligned_linear_acceleration;

        try
        {
            IValuelessConstant const *aligned_linear_velocity_goal_value_valueless_variable =
                    original_state->get_parameter_value(original_state->get_driving_agent_name() + ".aligned_linear_velocity.goal_value");
            IValuelessConstant const *aligned_linear_velocity_goal_duration_valueless_variable =
                    original_state->get_parameter_value(original_state->get_driving_agent_name() + ".aligned_linear_velocity.goal_duration");

            IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_goal_value_variable =
                    dynamic_cast<IConstant<FP_DATA_TYPE> const*>(aligned_linear_velocity_goal_value_valueless_variable);
            IConstant<temporal::Duration> const *aligned_linear_velocity_goal_duration_variable =
                    dynamic_cast<IConstant<temporal::Duration> const*>(aligned_linear_velocity_goal_duration_valueless_variable);

            FP_DATA_TYPE aligned_linear_velocity_goal_value = aligned_linear_velocity_goal_value_variable->get_value();
            temporal::Duration aligned_linear_velocity_goal_duration = aligned_linear_velocity_goal_duration_variable->get_value();

            FP_DATA_TYPE aligned_linear_velocity_error = aligned_linear_velocity_goal_value - aligned_linear_velocity;
            new_aligned_linear_acceleration = aligned_linear_velocity_error / std::max(aligned_linear_velocity_goal_duration.count(), time_step.count());
            new_aligned_linear_acceleration = std::min(new_aligned_linear_acceleration, MAX_ALIGNED_LINEAR_ACCELERATION);
            new_aligned_linear_acceleration = std::max(new_aligned_linear_acceleration, MIN_ALIGNED_LINEAR_ACCELERATION);

            IConstant<FP_DATA_TYPE> const *new_aligned_linear_acceleration_variable =
                        new BasicConstant(
                            modified_state->get_driving_agent_name(),
                            "aligned_linear_acceleration.indirect_actuation",
                            new_aligned_linear_acceleration);
            modified_state->set_aligned_linear_acceleration_variable(new_aligned_linear_acceleration_variable);
        }
        catch (std::out_of_range)
        {
            std::cerr << "Could not actuate aligned linear acceleration variable" << std::endl;
            modified_state->set_aligned_linear_acceleration_variable(
                        original_state->get_aligned_linear_acceleration_variable()->shallow_copy());
        }

        FP_DATA_TYPE aligned_linear_acceleration = original_state->get_aligned_linear_acceleration_variable()->get_value();

        FP_DATA_TYPE mean_aligned_linear_acceleration =
                (0.5 * (aligned_linear_acceleration + new_aligned_linear_acceleration) / steering_lookahead_steps) +
                ((steering_lookahead_steps - 1) * new_aligned_linear_acceleration / steering_lookahead_steps);

        FP_DATA_TYPE lookahead_distance_covered =
                aligned_linear_velocity * (time_step * steering_lookahead_steps).count() +
                0.5 * mean_aligned_linear_acceleration * std::pow((time_step * steering_lookahead_steps).count(), 2);

        geometry::Vec position = original_state->get_position_variable()->get_value();
        FP_DATA_TYPE rotation = original_state->get_rotation_variable()->get_value();

        map::ILaneArray<T_map_id> const *lanes;

        try
        {
            // TODO: Accomodate branching lanes
            lanes = map->get_encapsulating_lanes(position);

            if (lanes->count() > 0)
            {
                map::ILane<T_map_id> const *lane = (*lanes)[0];

                map::ILane<T_map_id> const *current_lane = lane;

                bool found_start = false;

                FP_DATA_TYPE distance_remaining = lookahead_distance_covered;

                geometry::Vec start_direction, end_direction, end_point;

                size_t i;

                FP_DATA_TYPE previous_distance_along_link = std::nanf("");

                while (true)
                {
                    geometry::Vecs const &left_boundary = current_lane->get_left_boundary();

                    for (i = 0;
                         i < left_boundary.cols() - 1;
                         ++i)
                    {
                        geometry::Vec on_boundary = left_boundary.col(i + 1) - left_boundary.col(i);
                        if (!found_start)
                        {
                            geometry::Vec from_boundary = position - left_boundary.col(i);
                            geometry::Vec on_boundary_normalised = on_boundary.normalized();
                            FP_DATA_TYPE distance_along_link = on_boundary_normalised.dot(from_boundary);

                            //std::cerr << "distance_along_link: " << distance_along_link << std::endl;
                            //std::cerr << "on_boundary.norm(): " << on_boundary.norm() << std::endl;

                            if (distance_along_link >= 0 && distance_along_link <= on_boundary.norm())
                            {
                                //std::cerr << "start found!" << std::endl;
                                found_start = true;
                                start_direction = on_boundary_normalised;
                                distance_remaining += distance_along_link;
                            }
                            else
                            {
                                if (!std::isnan(previous_distance_along_link))
                                {
                                    if ((distance_along_link >= 0 && previous_distance_along_link < 0) ||
                                            (distance_along_link < 0 && previous_distance_along_link >= 0))
                                    {
                                        //std::cerr << "Error sign changed" << std::endl;
                                        found_start = true;
                                        start_direction = on_boundary_normalised;
                                        distance_remaining += distance_along_link;
                                    }

                                    //if (std::abs(distance_along_link) > std::abs(previous_distance_along_link))
                                    //{
                                    //    std::cerr << "Error increased" << std::endl;
                                    //}
                                }

                                previous_distance_along_link = distance_along_link;
                            }
                        }

                        if (found_start)
                        {
                            distance_remaining -= on_boundary.norm();

                            if (distance_remaining < 0)
                            {
                                end_direction = on_boundary.normalized();
                                end_point = left_boundary.col(i);
                                break;
                            }
                        }
                    }

                    if (distance_remaining >= 0)
                    {
                        map::ILane<T_map_id> const *straight_fore_lane = current_lane->get_straight_fore_lane();
                        if (straight_fore_lane != nullptr)
                        {
                            //std::cerr << "new lane!" << std::endl;
                            current_lane = straight_fore_lane;
                        }
                        else
                        {
                            size_t last_index = left_boundary.cols() - 1;
                            end_direction = left_boundary.col(last_index) - left_boundary.col(last_index - 1);
                            end_point = left_boundary.col(last_index - 1);
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }

                if (!found_start)
                {
                    throw std::runtime_error("Could not find nearest lane boundary segment for driving agent");
                }

                // TODO: Remove previous start direction assignments
                start_direction = geometry::Vec(std::cos(rotation), std::sin(rotation));


                geometry::Vecs const &right_boundary = current_lane->get_right_boundary();

                geometry::Vec closest_right_boundary_point = right_boundary.col(0);
                FP_DATA_TYPE closest_right_boundary_point_distance =
                        (end_point - closest_right_boundary_point).norm();

                for (i = 1; i < right_boundary.cols(); ++i)
                {
                    geometry::Vec right_boundary_point = right_boundary.col(i);
                    FP_DATA_TYPE right_boundary_point_distance = (end_point - right_boundary_point).norm();
                    if (right_boundary_point_distance < closest_right_boundary_point_distance)
                    {
                        closest_right_boundary_point = right_boundary_point;
                        closest_right_boundary_point_distance = right_boundary_point_distance;
                    }
                }

                geometry::Vec lane_midpoint = (end_point + closest_right_boundary_point) / 2.0f;

                geometry::Vec midpoint_direction = (lane_midpoint - position).normalized();

                FP_DATA_TYPE lane_midpoint_angle_mag = std::acos(start_direction.dot(midpoint_direction));
                FP_DATA_TYPE lane_midpoint_angle;
                if (midpoint_direction.dot(trig_buff->get_rot_mat(lane_midpoint_angle_mag) * start_direction) >=
                        midpoint_direction.dot(trig_buff->get_rot_mat(-lane_midpoint_angle_mag) * start_direction))
                {
                    lane_midpoint_angle = lane_midpoint_angle_mag;
                }
                else
                {
                    lane_midpoint_angle = -lane_midpoint_angle_mag;
                }
                FP_DATA_TYPE lane_midpoint_steer = lane_midpoint_angle / lookahead_distance_covered;


                FP_DATA_TYPE lane_orientation_angle_mag = std::acos(start_direction.dot(end_direction));
                FP_DATA_TYPE lane_orientation_angle;
                if (end_direction.dot(trig_buff->get_rot_mat(lane_orientation_angle_mag) * start_direction) >=
                        end_direction.dot(trig_buff->get_rot_mat(-lane_orientation_angle_mag) * start_direction))
                {
                    lane_orientation_angle = lane_orientation_angle_mag;
                }
                else
                {
                    lane_orientation_angle = -lane_orientation_angle_mag;
                }
                FP_DATA_TYPE lane_orientation_steer = lane_orientation_angle / lookahead_distance_covered;


                FP_DATA_TYPE new_steer = (lane_midpoint_steer + lane_orientation_steer) / 2.0f;

                IConstant<FP_DATA_TYPE> const *new_steer_variable =
                            new BasicConstant(
                                modified_state->get_driving_agent_name(),
                                "steer.indirect_actuation",
                                new_steer);
                modified_state->set_steer_variable(new_steer_variable);
            }
            else
            {
                // Driving agent not on lane
                modified_state->set_steer_variable(original_state->get_steer_variable()->shallow_copy());
            }
        }
        catch (std::out_of_range)
        {
            std::cerr << "Could not actuate steer variable" << std::endl;
            modified_state->set_steer_variable(original_state->get_steer_variable()->shallow_copy());
        }
        catch (std::runtime_error)
        {
            modified_state->set_steer_variable(original_state->get_steer_variable()->shallow_copy());
        }

        delete lanes;
    }
};

}
}
}

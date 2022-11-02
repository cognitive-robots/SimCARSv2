
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicDrivingSimulator::BasicDrivingSimulator(std::shared_ptr<const IDrivingAgentController> controller) : controller(controller) {}

void BasicDrivingSimulator::simulate(std::shared_ptr<const IState> current_state, std::shared_ptr<IState> next_state, temporal::Duration time_step) const
{
    this->simulate_driving_scene(
                std::dynamic_pointer_cast<const IDrivingSceneState>(current_state),
                std::dynamic_pointer_cast<IDrivingSceneState>(next_state),
                time_step);
}

void BasicDrivingSimulator::simulate_driving_scene(
        std::shared_ptr<const IDrivingSceneState> current_state,
        std::shared_ptr<IDrivingSceneState> next_state,
        temporal::Duration time_step) const
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> current_driving_agent_states =
            current_state->get_driving_agent_states();

    std::shared_ptr<structures::IArray<std::shared_ptr<IDrivingAgentState>>> new_driving_agent_states(
                new structures::stl::STLStackArray<std::shared_ptr<IDrivingAgentState>>(current_driving_agent_states->count()));

    size_t i;
    for (i = 0; i < current_driving_agent_states->count(); ++i)
    {
        std::shared_ptr<const IDrivingAgentState> current_driving_agent_state = (*current_driving_agent_states)[i];
        std::shared_ptr<IDrivingAgentState> new_driving_agent_state(
                    new BasicDrivingAgentState(current_driving_agent_state));

        std::shared_ptr<IConstant<geometry::Vec>> external_linear_acceleration_variable(
                    new BasicConstant<geometry::Vec>(
                        new_driving_agent_state->get_driving_agent_name(),
                        "linear_acceleration.external",
                        geometry::Vec::Zero()));
        new_driving_agent_state->set_external_linear_acceleration_variable(external_linear_acceleration_variable);

        controller->modify_driving_agent_state(current_driving_agent_state, new_driving_agent_state);
        simulate_driving_agent(current_driving_agent_state, new_driving_agent_state, time_step);

        (*new_driving_agent_states)[i] = new_driving_agent_state;
    }

    /*
    for (i = 0; i < new_driving_agent_states->count(); ++i)
    {
        std::shared_ptr<IDrivingAgentState> new_driving_agent_state_1 = (*new_driving_agent_states)[i];

        geometry::Vec position_1 = new_driving_agent_state_1->get_position_variable()->get_value();
        FP_DATA_TYPE length_1 = new_driving_agent_state_1->get_bb_length_constant()->get_value();
        FP_DATA_TYPE width_1 = new_driving_agent_state_1->get_bb_width_constant()->get_value();
        FP_DATA_TYPE rotation_1 = new_driving_agent_state_1->get_rotation_variable()->get_value();

        geometry::ORect bounding_box_1(position_1, length_1, width_1, rotation_1);

        for (size_t j = i + 1; j < new_driving_agent_states->count(); ++j)
        {
            std::shared_ptr<IDrivingAgentState> new_driving_agent_state_2 = (*new_driving_agent_states)[j];

            geometry::Vec position_2 = new_driving_agent_state_2->get_position_variable()->get_value();
            FP_DATA_TYPE length_2 = new_driving_agent_state_2->get_bb_length_constant()->get_value();
            FP_DATA_TYPE width_2 = new_driving_agent_state_2->get_bb_width_constant()->get_value();
            FP_DATA_TYPE rotation_2 = new_driving_agent_state_2->get_rotation_variable()->get_value();

            geometry::ORect bounding_box_2(position_2, length_2, width_2, rotation_2);

            if (bounding_box_1.check_collision(bounding_box_2))
            {
                std::shared_ptr<const IDrivingAgentState> current_driving_agent_state_1 =
                        current_state->get_driving_agent_state(new_driving_agent_state_1->get_driving_agent_name());
                std::shared_ptr<const IDrivingAgentState> current_driving_agent_state_2 =
                        current_state->get_driving_agent_state(new_driving_agent_state_2->get_driving_agent_name());

                geometry::Vec direction = (position_2 - position_1).normalized();

                geometry::Vec acceleration_1 = new_driving_agent_state_1->get_linear_acceleration_variable()->get_value();
                geometry::Vec velocity_1 = new_driving_agent_state_1->get_linear_velocity_variable()->get_value();
                FP_DATA_TYPE mass_1 = length_1 * width_1;
                FP_DATA_TYPE collision_velocity_1 = velocity_1.dot(direction);

                geometry::Vec acceleration_2 = new_driving_agent_state_2->get_linear_acceleration_variable()->get_value();
                geometry::Vec velocity_2 = new_driving_agent_state_2->get_linear_velocity_variable()->get_value();
                FP_DATA_TYPE mass_2 = length_2 * width_2;
                FP_DATA_TYPE collision_velocity_2 = velocity_2.dot(direction);

                FP_DATA_TYPE resulting_velocity = ((mass_1 * collision_velocity_1) + (mass_2 * collision_velocity_2))
                        / (mass_1 + mass_2);

                geometry::Vec new_velocity_1 = velocity_1 + (resulting_velocity - collision_velocity_1) * direction;
                geometry::Vec new_velocity_2 = velocity_2 + (resulting_velocity - collision_velocity_2) * direction;

                geometry::Vec previous_velocity_1 = current_driving_agent_state_1->get_linear_velocity_variable()->get_value();
                geometry::Vec previous_velocity_2 = current_driving_agent_state_2->get_linear_velocity_variable()->get_value();

                geometry::Vec mean_acceleration_1 = (new_velocity_1 - previous_velocity_1) / time_step.count();
                geometry::Vec mean_acceleration_2 = (new_velocity_2 - previous_velocity_2) / time_step.count();

                geometry::Vec previous_acceleration_1 =
                        current_driving_agent_state_1->get_linear_acceleration_variable()->get_value();
                geometry::Vec previous_acceleration_2 =
                        current_driving_agent_state_2->get_linear_acceleration_variable()->get_value();

                geometry::Vec new_acceleration_1 = 2.0 * mean_acceleration_1 - previous_acceleration_1;
                geometry::Vec new_acceleration_2 = 2.0 * mean_acceleration_2 - previous_acceleration_2;

                geometry::Vec external_acceleration_1 = new_acceleration_1 - acceleration_1;
                geometry::Vec external_acceleration_2 = new_acceleration_2 - acceleration_2;

                std::shared_ptr<IConstant<geometry::Vec>> external_linear_acceleration_variable_1(
                            new BasicConstant<geometry::Vec>(
                                new_driving_agent_state_1->get_driving_agent_name(),
                                "linear_acceleration.external",
                                external_acceleration_1));
                new_driving_agent_state_1->set_external_linear_acceleration_variable(external_linear_acceleration_variable_1);
                simulate_driving_agent(current_driving_agent_state_1, new_driving_agent_state_1, time_step);

                std::shared_ptr<IConstant<geometry::Vec>> external_linear_acceleration_variable_2(
                            new BasicConstant<geometry::Vec>(
                                new_driving_agent_state_2->get_driving_agent_name(),
                                "linear_acceleration.external",
                                external_acceleration_2));
                new_driving_agent_state_2->set_external_linear_acceleration_variable(external_linear_acceleration_variable_2);
                simulate_driving_agent(current_driving_agent_state_2, new_driving_agent_state_2, time_step);
            }
        }
    }
    */

    for (i = 0; i < new_driving_agent_states->count(); ++i)
    {
        std::shared_ptr<IDrivingAgentState> new_driving_agent_state = (*new_driving_agent_states)[i];
        next_state->set_driving_agent_state(new_driving_agent_state);
    }
}

void BasicDrivingSimulator::simulate_driving_agent(std::shared_ptr<const IDrivingAgentState> current_state, std::shared_ptr<IDrivingAgentState> next_state, temporal::Duration time_step) const
{
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_acceleration_variable =
            current_state->get_aligned_linear_acceleration_variable();
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> new_aligned_linear_acceleration_variable =
            next_state->get_aligned_linear_acceleration_variable();
    FP_DATA_TYPE aligned_linear_acceleration = aligned_linear_acceleration_variable->get_value();
    FP_DATA_TYPE new_aligned_linear_acceleration = new_aligned_linear_acceleration_variable->get_value();

    std::shared_ptr<const IConstant<FP_DATA_TYPE>> steer_variable = current_state->get_steer_variable();
    std::shared_ptr<const IConstant<FP_DATA_TYPE>> new_steer_variable = next_state->get_steer_variable();
    FP_DATA_TYPE steer = steer_variable->get_value();
    FP_DATA_TYPE new_steer = new_steer_variable->get_value();

    std::shared_ptr<const IConstant<geometry::Vec>> new_external_linear_acceleration_variable =
            next_state->get_external_linear_acceleration_variable();
    geometry::Vec new_external_linear_acceleration = new_external_linear_acceleration_variable->get_value();

    std::shared_ptr<const IConstant<geometry::Vec>> linear_acceleration_variable =
            current_state->get_linear_acceleration_variable();
    geometry::Vec linear_acceleration = linear_acceleration_variable->get_value();

    std::shared_ptr<const IConstant<FP_DATA_TYPE>> aligned_linear_velocity_variable =
            current_state->get_aligned_linear_velocity_variable();
    FP_DATA_TYPE aligned_linear_velocity = aligned_linear_velocity_variable->get_value();

    std::shared_ptr<const IConstant<geometry::Vec>> linear_velocity_variable = current_state->get_linear_velocity_variable();
    geometry::Vec linear_velocity = linear_velocity_variable->get_value();

    std::shared_ptr<const IConstant<FP_DATA_TYPE>> angular_velocity_variable = current_state->get_angular_velocity_variable();
    FP_DATA_TYPE angular_velocity = angular_velocity_variable->get_value();

    std::shared_ptr<const IConstant<geometry::Vec>> position_variable = current_state->get_position_variable();
    geometry::Vec position = position_variable->get_value();

    std::shared_ptr<const IConstant<FP_DATA_TYPE>> rotation_variable = current_state->get_rotation_variable();
    FP_DATA_TYPE rotation = rotation_variable->get_value();


    FP_DATA_TYPE mean_aligned_linear_acceleration = (aligned_linear_acceleration + new_aligned_linear_acceleration) / 2.0f;

    FP_DATA_TYPE estimated_new_aligned_linear_velocity = aligned_linear_velocity + mean_aligned_linear_acceleration * time_step.count();

    FP_DATA_TYPE estimated_mean_aligned_linear_velocity = (aligned_linear_velocity + estimated_new_aligned_linear_velocity) / 2.0f;

    FP_DATA_TYPE new_angular_velocity = new_steer * estimated_mean_aligned_linear_velocity;

    std::shared_ptr<IConstant<FP_DATA_TYPE>> new_angular_velocity_variable(
                new BasicConstant<FP_DATA_TYPE>(next_state->get_driving_agent_name(), "angular_velocity.base", new_angular_velocity));
    next_state->set_angular_velocity_variable(new_angular_velocity_variable);

    FP_DATA_TYPE mean_angular_velocity = (angular_velocity + new_angular_velocity) / 2.0f;

    FP_DATA_TYPE new_rotation = rotation + mean_angular_velocity * time_step.count();

    std::shared_ptr<IConstant<FP_DATA_TYPE>> new_rotation_variable(
                new BasicConstant<FP_DATA_TYPE>(next_state->get_driving_agent_name(), "rotation.base", new_rotation));
    next_state->set_rotation_variable(new_rotation_variable);


    geometry::Vec new_linear_acceleration;
    new_linear_acceleration.x() = new_aligned_linear_acceleration * cos(new_rotation);
    new_linear_acceleration.y() = new_aligned_linear_acceleration * sin(new_rotation);
    new_linear_acceleration += new_external_linear_acceleration;

    std::shared_ptr<IConstant<geometry::Vec>> new_linear_acceleration_variable(
                new BasicConstant<geometry::Vec>(
                    next_state->get_driving_agent_name(), "linear_acceleration.base", new_linear_acceleration));
    next_state->set_linear_acceleration_variable(new_linear_acceleration_variable);

    geometry::Vec mean_linear_acceleration = (linear_acceleration + new_linear_acceleration) / 2.0f;

    geometry::Vec new_linear_velocity = linear_velocity + mean_linear_acceleration * time_step.count();

    std::shared_ptr<IConstant<geometry::Vec>> new_linear_velocity_variable(
                new BasicConstant<geometry::Vec>(next_state->get_driving_agent_name(), "linear_velocity.base", new_linear_velocity));
    next_state->set_linear_velocity_variable(new_linear_velocity_variable);

    std::shared_ptr<const geometry::TrigBuff> trig_buff = geometry::TrigBuff::get_instance();
    FP_DATA_TYPE new_aligned_linear_velocity = (trig_buff->get_rot_mat(-new_rotation) * new_linear_velocity).x();

    std::shared_ptr<IConstant<FP_DATA_TYPE>> new_aligned_linear_velocity_variable(
                new BasicConstant<FP_DATA_TYPE>(
                    next_state->get_driving_agent_name(), "aligned_linear_velocity.base", new_aligned_linear_velocity));
    next_state->set_aligned_linear_velocity_variable(new_aligned_linear_velocity_variable);

    geometry::Vec mean_linear_velocity = (linear_velocity + new_linear_velocity) / 2.0f;

    geometry::Vec new_position = position + mean_linear_velocity * time_step.count();

    std::shared_ptr<IConstant<geometry::Vec>> new_position_variable(
                new BasicConstant<geometry::Vec>(next_state->get_driving_agent_name(), "position.base", new_position));
    next_state->set_position_variable(new_position_variable);
}

}
}
}

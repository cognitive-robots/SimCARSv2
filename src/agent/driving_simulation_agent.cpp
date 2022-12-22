
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/basic_simulated_variable.hpp>
#include <ori/simcars/agent/driving_simulation_agent.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

DrivingSimulationAgent::DrivingSimulationAgent() {}

DrivingSimulationAgent::DrivingSimulationAgent(IDrivingAgent *driving_agent,
                                               ISimulationScene *simulation_scene,
                                               temporal::Time simulation_start_time,
                                               bool start_simulated,
                                               bool allow_late_start) :
    DrivingSimulationAgent(driving_agent, simulation_scene, simulation_start_time,
                           driving_agent->get_max_temporal_limit(), start_simulated,
                           allow_late_start)
{}

DrivingSimulationAgent::DrivingSimulationAgent(IDrivingAgent *driving_agent,
                                               ISimulationScene *simulation_scene,
                                               temporal::Time simulation_start_time,
                                               temporal::Time simulation_end_time,
                                               bool start_simulated,
                                               bool allow_late_start)
    : driving_agent(driving_agent)
{
    if (simulation_start_time < driving_agent->get_min_temporal_limit())
    {
        if (allow_late_start)
        {
            simulation_start_time = driving_agent->get_min_temporal_limit();
        }
        else
        {
            throw std::invalid_argument("Simulation start time is before earliest event for original agent");
        }
    }

    if (simulation_start_time > driving_agent->get_max_temporal_limit())
    {
        throw std::invalid_argument("Simulation start time is after latest event for original agent");
    }

    if (simulation_start_time > simulation_end_time)
    {
        throw std::invalid_argument("Simulation start time is after simulation end time");
    }

    if (start_simulated)
    {
        this->simulation_start_time = simulation_start_time;
    }
    else
    {
        this->simulation_start_time = simulation_end_time;
    }

    this->simulation_end_time = simulation_end_time;


    IVariable<geometry::Vec> *position_variable = driving_agent->get_mutable_position_variable();
    this->position_variable =
                new BasicSimulatedVariable(position_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<geometry::Vec> *linear_velocity_variable =
            driving_agent->get_mutable_linear_velocity_variable();
    this->linear_velocity_variable =
                new BasicSimulatedVariable(linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<FP_DATA_TYPE> *aligned_linear_velocity_variable =
            driving_agent->get_mutable_aligned_linear_velocity_variable();
    this->aligned_linear_velocity_variable =
                new BasicSimulatedVariable(aligned_linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<geometry::Vec> *linear_acceleration_variable =
            driving_agent->get_mutable_linear_acceleration_variable();
    this->linear_acceleration_variable =
                new BasicSimulatedVariable(linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<FP_DATA_TYPE> *aligned_linear_acceleration_variable =
            driving_agent->get_mutable_aligned_linear_acceleration_variable();
    this->aligned_linear_acceleration_variable =
                new BasicSimulatedVariable(aligned_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<geometry::Vec> *external_linear_acceleration_variable =
            driving_agent->get_mutable_external_linear_acceleration_variable();
    this->external_linear_acceleration_variable =
                new BasicSimulatedVariable(external_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<FP_DATA_TYPE> *rotation_variable =
            driving_agent->get_mutable_rotation_variable();
    this->rotation_variable =
                new BasicSimulatedVariable(rotation_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<FP_DATA_TYPE> *steer_variable =
            driving_agent->get_mutable_steer_variable();
    this->steer_variable =
                new BasicSimulatedVariable(steer_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<FP_DATA_TYPE> *angular_velocity_variable =
            driving_agent->get_mutable_angular_velocity_variable();
    this->angular_velocity_variable =
                new BasicSimulatedVariable(angular_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<temporal::Duration> *ttc_variable =
            driving_agent->get_mutable_ttc_variable();
    this->ttc_variable =
                new BasicSimulatedVariable(ttc_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());

    IVariable<temporal::Duration> *cumilative_collision_time_variable =
            driving_agent->get_mutable_cumilative_collision_time_variable();
    this->cumilative_collision_time_variable =
                new BasicSimulatedVariable(cumilative_collision_time_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
}

DrivingSimulationAgent::~DrivingSimulationAgent()
{
    delete position_variable;
    delete linear_velocity_variable;
    delete aligned_linear_velocity_variable;
    delete linear_acceleration_variable;
    delete aligned_linear_acceleration_variable;
    delete external_linear_acceleration_variable;
    delete rotation_variable;
    delete steer_variable;
    delete angular_velocity_variable;
    delete ttc_variable;
    delete cumilative_collision_time_variable;
}

std::string DrivingSimulationAgent::get_name() const
{
    return driving_agent->get_name();
}

// Not updated by simulation
geometry::Vec DrivingSimulationAgent::get_min_spatial_limits() const
{
    return driving_agent->get_min_spatial_limits();
}

// Not updated by simulation
geometry::Vec DrivingSimulationAgent::get_max_spatial_limits() const
{
    return driving_agent->get_max_spatial_limits();
}

temporal::Time DrivingSimulationAgent::get_min_temporal_limit() const
{
    return driving_agent->get_min_temporal_limit();
}

temporal::Time DrivingSimulationAgent::get_max_temporal_limit() const
{
    return this->simulation_end_time;
}

bool DrivingSimulationAgent::is_state_available(temporal::Time time) const
{
    return (time <= simulation_start_time &&
            driving_agent->is_state_available(time)) ||
            (time > simulation_start_time &&
             time <= simulation_end_time &&
             driving_agent->is_state_available(simulation_start_time));
}

structures::IArray<IValuelessConstant const*>* DrivingSimulationAgent::get_constant_parameters() const
{
    return driving_agent->get_constant_parameters();
}

IValuelessConstant const* DrivingSimulationAgent::get_constant_parameter(std::string const &constant_name) const
{
    return driving_agent->get_constant_parameter(constant_name);
}

structures::IArray<IValuelessVariable const*>* DrivingSimulationAgent::get_variable_parameters() const
{
    structures::stl::STLConcatArray<IValuelessVariable const*> *variables =
                new structures::stl::STLConcatArray<IValuelessVariable const*>(2);

    structures::IArray<IValuelessVariable const*> *simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable const*>(
    {
                    position_variable,
                    linear_velocity_variable,
                    aligned_linear_velocity_variable,
                    linear_acceleration_variable,
                    aligned_linear_acceleration_variable,
                    external_linear_acceleration_variable,
                    rotation_variable,
                    steer_variable,
                    angular_velocity_variable,
                    ttc_variable,
                    cumilative_collision_time_variable
                });
    variables->get_array(0) = simulated_variables;

    structures::IArray<IValuelessVariable const*> *unfiltered_non_simulated_variables =
            driving_agent->get_variable_parameters();
    structures::IStackArray<IValuelessVariable const*> *filtered_non_simulated_variables =
            new structures::stl::STLStackArray<IValuelessVariable const*>;

    bool found_variable;
    size_t i, j;
    for (i = 0; i < unfiltered_non_simulated_variables->count(); ++i)
    {
        found_variable = false;

        for (j = 0; j < simulated_variables->count(); ++j)
        {
            if ((*simulated_variables)[j]->get_parameter_name() ==
                    (*unfiltered_non_simulated_variables)[i]->get_parameter_name())
            {
                found_variable = true;
                break;
            }
        }

        if (!found_variable)
        {
            filtered_non_simulated_variables->push_back((*unfiltered_non_simulated_variables)[i]);
        }
    }

    variables->get_array(1) = filtered_non_simulated_variables;

    delete unfiltered_non_simulated_variables;

    return variables;
}

IValuelessVariable const* DrivingSimulationAgent::get_variable_parameter(std::string const &variable_name) const
{
    if (variable_name == this->get_name() + ".position.base")
    {
        return position_variable;
    }
    else if (variable_name == this->get_name() + ".linear_velocity.base")
    {
        return linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_velocity.base")
    {
        return aligned_linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.base")
    {
        return linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_acceleration.indirect_actuation")
    {
        return aligned_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.external")
    {
        return external_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".rotation.base")
    {
        return rotation_variable;
    }
    else if (variable_name == this->get_name() + ".steer.indirect_actuation")
    {
        return steer_variable;
    }
    else if (variable_name == this->get_name() + ".angular_velocity.base")
    {
        return angular_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".ttc.base")
    {
        return ttc_variable;
    }
    else if (variable_name == this->get_name() + ".cumilative_collision_time.base")
    {
        return cumilative_collision_time_variable;
    }
    else
    {
        return driving_agent->get_variable_parameter(variable_name);
    }
}

structures::IArray<IValuelessEvent const*>* DrivingSimulationAgent::get_events() const
{
    structures::stl::STLConcatArray<IValuelessEvent const*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent const*>(11);

    events->get_array(0) = position_variable->get_valueless_events();
    events->get_array(1) = linear_velocity_variable->get_valueless_events();
    events->get_array(2) = aligned_linear_velocity_variable->get_valueless_events();
    events->get_array(3) = linear_acceleration_variable->get_valueless_events();
    events->get_array(4) = aligned_linear_acceleration_variable->get_valueless_events();
    events->get_array(5) = external_linear_acceleration_variable->get_valueless_events();
    events->get_array(6) = rotation_variable->get_valueless_events();
    events->get_array(7) = steer_variable->get_valueless_events();
    events->get_array(8) = angular_velocity_variable->get_valueless_events();
    events->get_array(9) = ttc_variable->get_valueless_events();
    events->get_array(10) = cumilative_collision_time_variable->get_valueless_events();

    return events;
}

/*
 * NOTE: This should not be called directly by external code, as the simulation scene will not have a pointer to the
 * resulting copy and thus any new simulation data will not be propogated to the copy.
 */
IDrivingSimulationAgent* DrivingSimulationAgent::driving_simulation_agent_deep_copy(IDrivingSimulationScene *driving_simulation_scene) const
{
    DrivingSimulationAgent *driving_agent = new DrivingSimulationAgent();

    driving_agent->driving_agent = this->driving_agent;

    driving_agent->simulation_start_time = this->simulation_start_time;
    driving_agent->simulation_end_time = this->simulation_end_time;

    driving_agent->position_variable = this->position_variable->simulated_variable_deep_copy();
    driving_agent->linear_velocity_variable = this->linear_velocity_variable->simulated_variable_deep_copy();
    driving_agent->aligned_linear_velocity_variable = this->aligned_linear_velocity_variable->simulated_variable_deep_copy();
    driving_agent->linear_acceleration_variable = this->linear_acceleration_variable->simulated_variable_deep_copy();
    driving_agent->aligned_linear_acceleration_variable = this->aligned_linear_acceleration_variable->simulated_variable_deep_copy();
    driving_agent->external_linear_acceleration_variable = this->external_linear_acceleration_variable->simulated_variable_deep_copy();
    driving_agent->rotation_variable = this->rotation_variable->simulated_variable_deep_copy();
    driving_agent->steer_variable = this->steer_variable->simulated_variable_deep_copy();
    driving_agent->angular_velocity_variable = this->angular_velocity_variable->simulated_variable_deep_copy();
    driving_agent->ttc_variable = this->ttc_variable->simulated_variable_deep_copy();
    driving_agent->cumilative_collision_time_variable = this->cumilative_collision_time_variable->simulated_variable_deep_copy();

    if (driving_simulation_scene == nullptr)
    {
        driving_agent->driving_simulation_scene = this->driving_simulation_scene;
    }
    else
    {
        driving_agent->driving_simulation_scene = driving_simulation_scene;
    }

    return driving_agent;
}

IConstant<uint32_t> const* DrivingSimulationAgent::get_id_constant() const
{
    return driving_agent->get_id_constant();
}

IConstant<bool> const* DrivingSimulationAgent::get_ego_constant() const
{
    return driving_agent->get_ego_constant();
}

IConstant<FP_DATA_TYPE> const* DrivingSimulationAgent::get_bb_length_constant() const
{
    return driving_agent->get_bb_length_constant();
}

IConstant<FP_DATA_TYPE> const* DrivingSimulationAgent::get_bb_width_constant() const
{
    return driving_agent->get_bb_width_constant();
}

IConstant<DrivingAgentClass> const* DrivingSimulationAgent::get_driving_agent_class_constant() const
{
    return driving_agent->get_driving_agent_class_constant();
}

IVariable<geometry::Vec> const* DrivingSimulationAgent::get_position_variable() const
{
    return position_variable;
}

IVariable<geometry::Vec> const* DrivingSimulationAgent::get_linear_velocity_variable() const
{
    return linear_velocity_variable;
}

IVariable<FP_DATA_TYPE> const* DrivingSimulationAgent::get_aligned_linear_velocity_variable() const
{
    return aligned_linear_velocity_variable;
}

IVariable<geometry::Vec> const* DrivingSimulationAgent::get_linear_acceleration_variable() const
{
    return linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE> const* DrivingSimulationAgent::get_aligned_linear_acceleration_variable() const
{
    return aligned_linear_acceleration_variable;
}

IVariable<geometry::Vec> const* DrivingSimulationAgent::get_external_linear_acceleration_variable() const
{
    return external_linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE> const* DrivingSimulationAgent::get_rotation_variable() const
{
    return rotation_variable;
}

IVariable<FP_DATA_TYPE> const* DrivingSimulationAgent::get_steer_variable() const
{
    return steer_variable;
}

IVariable<FP_DATA_TYPE> const* DrivingSimulationAgent::get_angular_velocity_variable() const
{
    return angular_velocity_variable;
}

IVariable<temporal::Duration> const* DrivingSimulationAgent::get_ttc_variable() const
{
    return ttc_variable;
}

IVariable<temporal::Duration> const* DrivingSimulationAgent::get_cumilative_collision_time_variable() const
{
    return cumilative_collision_time_variable;
}

IDrivingSimulationScene const* DrivingSimulationAgent::get_driving_simulation_scene() const
{
    return this->driving_simulation_scene;
}

void DrivingSimulationAgent::begin_simulation(temporal::Time simulation_start_time) const
{
    if (this->simulation_start_time == this->simulation_end_time &&
            simulation_start_time <= driving_agent->get_max_temporal_limit())
    {
        this->simulation_start_time = simulation_start_time;

        position_variable->begin_simulation(simulation_start_time);
        linear_velocity_variable->begin_simulation(simulation_start_time);
        aligned_linear_velocity_variable->begin_simulation(simulation_start_time);
        linear_acceleration_variable->begin_simulation(simulation_start_time);
        aligned_linear_acceleration_variable->begin_simulation(simulation_start_time);
        external_linear_acceleration_variable->begin_simulation(simulation_start_time);
        rotation_variable->begin_simulation(simulation_start_time);
        steer_variable->begin_simulation(simulation_start_time);
        angular_velocity_variable->begin_simulation(simulation_start_time);
        ttc_variable->begin_simulation(simulation_start_time);
        cumilative_collision_time_variable->begin_simulation(simulation_start_time);
    }
}

structures::IArray<IValuelessConstant*>* DrivingSimulationAgent::get_mutable_constant_parameters()
{
    return driving_agent->get_mutable_constant_parameters();
}

IValuelessConstant* DrivingSimulationAgent::get_mutable_constant_parameter(std::string const &constant_name)
{
    return driving_agent->get_mutable_constant_parameter(constant_name);
}

structures::IArray<IValuelessVariable*>* DrivingSimulationAgent::get_mutable_variable_parameters()
{
    structures::stl::STLConcatArray<IValuelessVariable*> *variables =
                new structures::stl::STLConcatArray<IValuelessVariable*>(2);

    structures::IArray<IValuelessVariable*> *simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable*>(
    {
                    position_variable,
                    linear_velocity_variable,
                    aligned_linear_velocity_variable,
                    linear_acceleration_variable,
                    aligned_linear_acceleration_variable,
                    external_linear_acceleration_variable,
                    rotation_variable,
                    steer_variable,
                    angular_velocity_variable,
                    ttc_variable,
                    cumilative_collision_time_variable
                });
    variables->get_array(0) = simulated_variables;

    structures::IArray<IValuelessVariable*> *unfiltered_non_simulated_variables =
            driving_agent->get_mutable_variable_parameters();
    structures::IStackArray<IValuelessVariable*> *filtered_non_simulated_variables =
            new structures::stl::STLStackArray<IValuelessVariable*>;

    bool found_variable;
    size_t i, j;
    for (i = 0; i < unfiltered_non_simulated_variables->count(); ++i)
    {
        found_variable = false;

        for (j = 0; j < simulated_variables->count(); ++j)
        {
            if ((*simulated_variables)[j]->get_parameter_name() ==
                    (*unfiltered_non_simulated_variables)[i]->get_parameter_name())
            {
                found_variable = true;
                break;
            }
        }

        if (!found_variable)
        {
            filtered_non_simulated_variables->push_back((*unfiltered_non_simulated_variables)[i]);
        }
    }

    variables->get_array(1) = filtered_non_simulated_variables;

    delete unfiltered_non_simulated_variables;

    return variables;
}

IValuelessVariable* DrivingSimulationAgent::get_mutable_variable_parameter(std::string const &variable_name)
{
    if (variable_name == this->get_name() + ".position.base")
    {
        return position_variable;
    }
    else if (variable_name == this->get_name() + ".linear_velocity.base")
    {
        return linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_velocity.base")
    {
        return aligned_linear_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.base")
    {
        return linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".aligned_linear_acceleration.indirect_actuation")
    {
        return aligned_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".linear_acceleration.external")
    {
        return external_linear_acceleration_variable;
    }
    else if (variable_name == this->get_name() + ".rotation.base")
    {
        return rotation_variable;
    }
    else if (variable_name == this->get_name() + ".steer.indirect_actuation")
    {
        return steer_variable;
    }
    else if (variable_name == this->get_name() + ".angular_velocity.base")
    {
        return angular_velocity_variable;
    }
    else if (variable_name == this->get_name() + ".ttc.base")
    {
        return ttc_variable;
    }
    else if (variable_name == this->get_name() + ".cumilative_collision_time.base")
    {
        return cumilative_collision_time_variable;
    }
    else
    {
        return driving_agent->get_mutable_variable_parameter(variable_name);
    }
}

structures::IArray<IValuelessEvent*>* DrivingSimulationAgent::get_mutable_events()
{
    structures::stl::STLConcatArray<IValuelessEvent*> *events =
            new structures::stl::STLConcatArray<IValuelessEvent*>(11);

    events->get_array(0) = position_variable->get_mutable_valueless_events();
    events->get_array(1) = linear_velocity_variable->get_mutable_valueless_events();
    events->get_array(2) = aligned_linear_velocity_variable->get_mutable_valueless_events();
    events->get_array(3) = linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(4) = aligned_linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(5) = external_linear_acceleration_variable->get_mutable_valueless_events();
    events->get_array(6) = rotation_variable->get_mutable_valueless_events();
    events->get_array(7) = steer_variable->get_mutable_valueless_events();
    events->get_array(8) = angular_velocity_variable->get_mutable_valueless_events();
    events->get_array(9) = ttc_variable->get_mutable_valueless_events();
    events->get_array(10) = cumilative_collision_time_variable->get_mutable_valueless_events();

    return events;
}

IConstant<uint32_t>* DrivingSimulationAgent::get_mutable_id_constant()
{
    return driving_agent->get_mutable_id_constant();
}

IConstant<bool>* DrivingSimulationAgent::get_mutable_ego_constant()
{
    return driving_agent->get_mutable_ego_constant();
}

IConstant<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_bb_length_constant()
{
    return driving_agent->get_mutable_bb_length_constant();
}

IConstant<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_bb_width_constant()
{
    return driving_agent->get_mutable_bb_width_constant();
}

IConstant<DrivingAgentClass>* DrivingSimulationAgent::get_mutable_driving_agent_class_constant()
{
    return driving_agent->get_mutable_driving_agent_class_constant();
}

IVariable<geometry::Vec>* DrivingSimulationAgent::get_mutable_position_variable()
{
    return position_variable;
}

IVariable<geometry::Vec>* DrivingSimulationAgent::get_mutable_linear_velocity_variable()
{
    return linear_velocity_variable;
}

IVariable<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_aligned_linear_velocity_variable()
{
    return aligned_linear_velocity_variable;
}

IVariable<geometry::Vec>* DrivingSimulationAgent::get_mutable_linear_acceleration_variable()
{
    return linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_aligned_linear_acceleration_variable()
{
    return aligned_linear_acceleration_variable;
}

IVariable<geometry::Vec>* DrivingSimulationAgent::get_mutable_external_linear_acceleration_variable()
{
    return external_linear_acceleration_variable;
}

IVariable<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_rotation_variable()
{
    return rotation_variable;
}

IVariable<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_steer_variable()
{
    return steer_variable;
}

IVariable<FP_DATA_TYPE>* DrivingSimulationAgent::get_mutable_angular_velocity_variable()
{
    return angular_velocity_variable;
}

IVariable<temporal::Duration>* DrivingSimulationAgent::get_mutable_ttc_variable()
{
    return ttc_variable;
}

IVariable<temporal::Duration>* DrivingSimulationAgent::get_mutable_cumilative_collision_time_variable()
{
    return cumilative_collision_time_variable;
}

}
}
}

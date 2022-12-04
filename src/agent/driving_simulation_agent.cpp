
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/basic_constant.hpp>
#include <ori/simcars/agent/simulated_variable.hpp>
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
    SimulatedVariable<geometry::Vec> *simulated_position_variable =
                new SimulatedVariable(position_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_position_variable->get_full_name(), simulated_position_variable);

    IVariable<geometry::Vec> *linear_velocity_variable =
            driving_agent->get_mutable_linear_velocity_variable();
    SimulatedVariable<geometry::Vec> *simulated_linear_velocity_variable =
                new SimulatedVariable(linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_linear_velocity_variable->get_full_name(), simulated_linear_velocity_variable);

    IVariable<FP_DATA_TYPE> *aligned_linear_velocity_variable =
            driving_agent->get_mutable_aligned_linear_velocity_variable();
    SimulatedVariable<FP_DATA_TYPE> *simulated_aligned_linear_velocity_variable =
                new SimulatedVariable(aligned_linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_aligned_linear_velocity_variable->get_full_name(), simulated_aligned_linear_velocity_variable);

    IVariable<geometry::Vec> *linear_acceleration_variable =
            driving_agent->get_mutable_linear_acceleration_variable();
    SimulatedVariable<geometry::Vec> *simulated_linear_acceleration_variable =
                new SimulatedVariable(linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_linear_acceleration_variable->get_full_name(), simulated_linear_acceleration_variable);

    IVariable<FP_DATA_TYPE> *aligned_linear_acceleration_variable =
            driving_agent->get_mutable_aligned_linear_acceleration_variable();
    SimulatedVariable<FP_DATA_TYPE> *simulated_aligned_linear_acceleration_variable =
                new SimulatedVariable(aligned_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_aligned_linear_acceleration_variable->get_full_name(), simulated_aligned_linear_acceleration_variable);

    IVariable<geometry::Vec> *external_linear_acceleration_variable =
            driving_agent->get_mutable_external_linear_acceleration_variable();
    SimulatedVariable<geometry::Vec> *simulated_external_linear_acceleration_variable =
                new SimulatedVariable(external_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_external_linear_acceleration_variable->get_full_name(), simulated_external_linear_acceleration_variable);

    IVariable<FP_DATA_TYPE> *rotation_variable =
            driving_agent->get_mutable_rotation_variable();
    SimulatedVariable<FP_DATA_TYPE> *simulated_rotation_variable =
                new SimulatedVariable(rotation_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_rotation_variable->get_full_name(), simulated_rotation_variable);

    IVariable<FP_DATA_TYPE> *steer_variable =
            driving_agent->get_mutable_steer_variable();
    SimulatedVariable<FP_DATA_TYPE> *simulated_steer_variable =
                new SimulatedVariable(steer_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_steer_variable->get_full_name(), simulated_steer_variable);

    IVariable<FP_DATA_TYPE> *angular_velocity_variable =
            driving_agent->get_mutable_angular_velocity_variable();
    SimulatedVariable<FP_DATA_TYPE> *simulated_angular_velocity_variable =
                new SimulatedVariable(angular_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated, simulation_scene->get_time_step());
    this->simulated_variable_dict.update(simulated_angular_velocity_variable->get_full_name(), simulated_angular_velocity_variable);


    structures::IArray<IValuelessVariable*> const *variables =
            driving_agent->get_mutable_variable_parameters();
    for (size_t i = 0; i < variables->count(); ++i)
    {
        if (!this->simulated_variable_dict.contains((*variables)[i]->get_full_name()))
        {
            this->non_simulated_variable_dict.update((*variables)[i]->get_full_name(), (*variables)[i]);
        }
    }
    delete variables;
}

DrivingSimulationAgent::~DrivingSimulationAgent()
{
    structures::IArray<ISimulatedValuelessVariable*> const *simulated_variables = simulated_variable_dict.get_values();

    for (size_t i = 0; i < simulated_variables->count(); ++i)
    {
        delete (*simulated_variables)[i];
    }
}

std::string DrivingSimulationAgent::get_name() const
{
    return this->driving_agent->get_name();
}

// Not updated by simulation
geometry::Vec DrivingSimulationAgent::get_min_spatial_limits() const
{
    return this->driving_agent->get_min_spatial_limits();
}

// Not updated by simulation
geometry::Vec DrivingSimulationAgent::get_max_spatial_limits() const
{
    return this->driving_agent->get_max_spatial_limits();
}

temporal::Time DrivingSimulationAgent::get_min_temporal_limit() const
{
    return this->driving_agent->get_min_temporal_limit();
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
    return this->driving_agent->get_constant_parameters();
}

IValuelessConstant const* DrivingSimulationAgent::get_constant_parameter(std::string const &constant_name) const
{
    return this->driving_agent->get_constant_parameter(constant_name);
}

structures::IArray<IValuelessVariable const*>* DrivingSimulationAgent::get_variable_parameters() const
{
    structures::stl::STLConcatArray<IValuelessVariable const*> *variables =
                new structures::stl::STLConcatArray<IValuelessVariable const*>(2);

    structures::IArray<IValuelessVariable const*> *non_simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable const*>(non_simulated_variable_dict.count());
    cast_array(*non_simulated_variable_dict.get_values(), *non_simulated_variables);
    variables->get_array(0) = non_simulated_variables;

    structures::IArray<IValuelessVariable const*> *simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable const*>(simulated_variable_dict.count());
    cast_array<ISimulatedValuelessVariable*, IValuelessVariable const*>(
                *(simulated_variable_dict.get_values()), *simulated_variables);
    variables->get_array(1) = simulated_variables;

    return variables;
}

IValuelessVariable const* DrivingSimulationAgent::get_variable_parameter(std::string const &variable_name) const
{
    if (non_simulated_variable_dict.contains(variable_name))
    {
        return non_simulated_variable_dict[variable_name];
    }
    else
    {
        return simulated_variable_dict[variable_name];
    }
}

structures::IArray<IValuelessEvent const*>* DrivingSimulationAgent::get_events() const
{
    structures::IArray<std::string> const *non_simulated_variable_names = non_simulated_variable_dict.get_keys();
    structures::IArray<std::string> const *simulated_variable_names = simulated_variable_dict.get_keys();

    structures::stl::STLConcatArray<IValuelessEvent const*> *events =
                new structures::stl::STLConcatArray<IValuelessEvent const*>(
                    non_simulated_variable_names->count() + simulated_variable_names->count());

    size_t i;

    for(i = 0; i < non_simulated_variable_names->count(); ++i)
    {
        events->get_array(i) = non_simulated_variable_dict[(*non_simulated_variable_names)[i]]->get_valueless_events();
    }

    for(i = 0; i < simulated_variable_names->count(); ++i)
    {
        events->get_array(non_simulated_variable_names->count() + i) = simulated_variable_dict[(*simulated_variable_names)[i]]->get_valueless_events();
    }

    return events;
}

/*
 * NOTE: This should not be called directly by external code, as the simulation scene will not have a pointer to the
 * resulting copy and thus any new simulation data will not be propogated to the copy.
 */
IDrivingAgent* DrivingSimulationAgent::driving_agent_deep_copy() const
{
    DrivingSimulationAgent *driving_agent = new DrivingSimulationAgent();

    driving_agent->driving_agent = this->driving_agent;
    driving_agent->simulation_start_time = this->simulation_start_time;
    driving_agent->simulation_end_time = this->simulation_end_time;

    size_t i;

    structures::IArray<std::string> const *non_simulated_variable_names = non_simulated_variable_dict.get_keys();
    for(i = 0; i < non_simulated_variable_names->count(); ++i)
    {
        driving_agent->non_simulated_variable_dict.update((*non_simulated_variable_names)[i], non_simulated_variable_dict[(*non_simulated_variable_names)[i]]->valueless_deep_copy());
    }

    structures::IArray<std::string> const *simulated_variable_names = simulated_variable_dict.get_keys();
    for(i = 0; i < simulated_variable_names->count(); ++i)
    {
        driving_agent->simulated_variable_dict.update((*simulated_variable_names)[i], simulated_variable_dict[(*simulated_variable_names)[i]]->simulated_valueless_deep_copy());
    }

    return driving_agent;
}

void DrivingSimulationAgent::begin_simulation(temporal::Time simulation_start_time) const
{
    if (this->simulation_start_time == this->simulation_end_time &&
            simulation_start_time <= driving_agent->get_max_temporal_limit())
    {
        this->simulation_start_time = simulation_start_time;

        structures::IArray<ISimulatedValuelessVariable*> const *simulated_variables =
                simulated_variable_dict.get_values();
        for(size_t i = 0; i < simulated_variables->count(); ++i)
        {
            (*simulated_variables)[i]->begin_simulation(simulation_start_time);
        }
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

    variables->get_array(0) =
                new structures::stl::STLStackArray<IValuelessVariable*>(
                    non_simulated_variable_dict.get_values());

    structures::IArray<IValuelessVariable*> *simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable*>(simulated_variable_dict.count());
    cast_array<ISimulatedValuelessVariable*, IValuelessVariable*>(
                *(simulated_variable_dict.get_values()), *simulated_variables);
    variables->get_array(1) = simulated_variables;

    return variables;
}

IValuelessVariable* DrivingSimulationAgent::get_mutable_variable_parameter(std::string const &variable_name)
{
    if (non_simulated_variable_dict.contains(variable_name))
    {
        return non_simulated_variable_dict[variable_name];
    }
    else
    {
        return simulated_variable_dict[variable_name];
    }
}

structures::IArray<IValuelessEvent*>* DrivingSimulationAgent::get_mutable_events()
{
    structures::IArray<std::string> const *non_simulated_variable_names = non_simulated_variable_dict.get_keys();
    structures::IArray<std::string> const *simulated_variable_names = simulated_variable_dict.get_keys();

    structures::stl::STLConcatArray<IValuelessEvent*> *events =
                new structures::stl::STLConcatArray<IValuelessEvent*>(
                    non_simulated_variable_names->count() + simulated_variable_names->count());

    size_t i;

    for(i = 0; i < non_simulated_variable_names->count(); ++i)
    {
        events->get_array(i) = non_simulated_variable_dict[(*non_simulated_variable_names)[i]]->get_mutable_valueless_events();
    }

    for(i = 0; i < simulated_variable_names->count(); ++i)
    {
        events->get_array(non_simulated_variable_names->count() + i) = simulated_variable_dict[(*simulated_variable_names)[i]]->get_mutable_valueless_events();
    }

    return events;
}

}
}
}

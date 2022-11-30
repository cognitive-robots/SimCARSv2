
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

DrivingSimulationAgent::DrivingSimulationAgent(IDrivingAgent const *driving_agent,
                                               ISimulationScene const *simulation_scene,
                                               temporal::Time simulation_start_time,
                                               bool start_simulated,
                                               bool allow_late_start) :
    DrivingSimulationAgent(driving_agent, simulation_scene, simulation_start_time,
                           driving_agent->get_max_temporal_limit(), start_simulated,
                           allow_late_start)
{}

DrivingSimulationAgent::DrivingSimulationAgent(IDrivingAgent const *driving_agent,
                                               ISimulationScene const *simulation_scene,
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

    this->max_temporal_limit = simulation_end_time;


    IVariable<geometry::Vec> const *position_variable = driving_agent->get_position_variable();
    SimulatedVariable<geometry::Vec> const *simulated_position_variable =
                new SimulatedVariable(position_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_position_variable->get_full_name(), simulated_position_variable);

    IVariable<geometry::Vec> const *linear_velocity_variable =
            driving_agent->get_linear_velocity_variable();
    SimulatedVariable<geometry::Vec> const *simulated_linear_velocity_variable =
                new SimulatedVariable(linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_linear_velocity_variable->get_full_name(), simulated_linear_velocity_variable);

    IVariable<FP_DATA_TYPE> const *aligned_linear_velocity_variable =
            driving_agent->get_aligned_linear_velocity_variable();
    SimulatedVariable<FP_DATA_TYPE> const *simulated_aligned_linear_velocity_variable =
                new SimulatedVariable(aligned_linear_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_aligned_linear_velocity_variable->get_full_name(), simulated_aligned_linear_velocity_variable);

    IVariable<geometry::Vec> const *linear_acceleration_variable =
            driving_agent->get_linear_acceleration_variable();
    SimulatedVariable<geometry::Vec> const *simulated_linear_acceleration_variable =
                new SimulatedVariable(linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_linear_acceleration_variable->get_full_name(), simulated_linear_acceleration_variable);

    IVariable<FP_DATA_TYPE> const *aligned_linear_acceleration_variable =
            driving_agent->get_aligned_linear_acceleration_variable();
    SimulatedVariable<FP_DATA_TYPE> const *simulated_aligned_linear_acceleration_variable =
                new SimulatedVariable(aligned_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_aligned_linear_acceleration_variable->get_full_name(), simulated_aligned_linear_acceleration_variable);

    IVariable<geometry::Vec> const *external_linear_acceleration_variable =
            driving_agent->get_external_linear_acceleration_variable();
    SimulatedVariable<geometry::Vec> const *simulated_external_linear_acceleration_variable =
                new SimulatedVariable(external_linear_acceleration_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_external_linear_acceleration_variable->get_full_name(), simulated_external_linear_acceleration_variable);

    IVariable<FP_DATA_TYPE> const *rotation_variable =
            driving_agent->get_rotation_variable();
    SimulatedVariable<FP_DATA_TYPE> const *simulated_rotation_variable =
                new SimulatedVariable(rotation_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_rotation_variable->get_full_name(), simulated_rotation_variable);

    IVariable<FP_DATA_TYPE> const *steer_variable =
            driving_agent->get_steer_variable();
    SimulatedVariable<FP_DATA_TYPE> const *simulated_steer_variable =
                new SimulatedVariable(steer_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_steer_variable->get_full_name(), simulated_steer_variable);

    IVariable<FP_DATA_TYPE> const *angular_velocity_variable =
            driving_agent->get_angular_velocity_variable();
    SimulatedVariable<FP_DATA_TYPE> const *simulated_angular_velocity_variable =
                new SimulatedVariable(angular_velocity_variable, simulation_scene, simulation_start_time,
                                      simulation_end_time, start_simulated);
    this->simulated_variable_dict.update(simulated_angular_velocity_variable->get_full_name(), simulated_angular_velocity_variable);


    structures::IArray<IValuelessVariable const*> const *variables =
            driving_agent->get_variable_parameters();
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
    structures::IArray<ISimulatedValuelessVariable const*> const *simulated_variables = simulated_variable_dict.get_values();

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
    return this->max_temporal_limit;
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

    variables->get_array(0) =
                new structures::stl::STLStackArray<IValuelessVariable const*>(
                    non_simulated_variable_dict.get_values());

    structures::IArray<IValuelessVariable const*> *simulated_variables =
                new structures::stl::STLStackArray<IValuelessVariable const*>(simulated_variable_dict.count());
    cast_array<ISimulatedValuelessVariable const*, IValuelessVariable const*>(
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
    driving_agent->max_temporal_limit = this->max_temporal_limit;

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

IDrivingAgentState* DrivingSimulationAgent::get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const
{
    IDrivingAgentState *driving_agent_state =
            driving_agent->get_driving_agent_state(time, false);

    try
    {
        driving_agent_state->set_position_variable(
                    this->get_position_variable()->get_event(time));
        driving_agent_state->set_linear_velocity_variable(
                    this->get_linear_velocity_variable()->get_event(time));
        driving_agent_state->set_aligned_linear_velocity_variable(
                    this->get_aligned_linear_velocity_variable()->get_event(time));
        driving_agent_state->set_linear_acceleration_variable(
                    this->get_linear_acceleration_variable()->get_event(time));
        driving_agent_state->set_aligned_linear_acceleration_variable(
                    this->get_aligned_linear_acceleration_variable()->get_event(time));
        driving_agent_state->set_external_linear_acceleration_variable(
                    this->get_external_linear_acceleration_variable()->get_event(time));
        driving_agent_state->set_rotation_variable(
                    this->get_rotation_variable()->get_event(time));
        driving_agent_state->set_steer_variable(
                    this->get_steer_variable()->get_event(time));
        driving_agent_state->set_angular_velocity_variable(
                    this->get_angular_velocity_variable()->get_event(time));
    }
    catch (std::out_of_range const &e)
    {
        if (throw_on_out_of_range)
        {
            delete driving_agent_state;
            throw e;
        }
    }

    return driving_agent_state;
}

void DrivingSimulationAgent::propogate(temporal::Time time, IDrivingAgentState const *state) const
{
    structures::IArray<ISimulatedValuelessVariable const*> const *simulated_variables =
            simulated_variable_dict.get_values();
    for(size_t i = 0; i < simulated_variables->count(); ++i)
    {
        (*simulated_variables)[i]->simulation_update(time, state);
    }
}

void DrivingSimulationAgent::begin_simulation(temporal::Time simulation_start_time) const
{
    structures::IArray<ISimulatedValuelessVariable const*> const *simulated_variables =
            simulated_variable_dict.get_values();
    for(size_t i = 0; i < simulated_variables->count(); ++i)
    {
        (*simulated_variables)[i]->begin_simulation(simulation_start_time);
    }
}

}
}
}

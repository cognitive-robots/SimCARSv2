
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/causal/necessary_fp_goal_causal_link_tester.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE NecessaryFPGoalCausalLinkTester::calculate_expected_reward(
        agent::IScene const *scene, agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *cause,
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *effect,
        structures::IArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> const *alternative_actions,
        bool original_scene) const
{
    FP_DATA_TYPE effect_reward_minimum = 1.0f;

    agent::IEntity const *effect_entity = scene->get_entity(effect->get_entity_name());

    if (effect->get_time() == effect_entity->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential effect event cannot be an initialising event");
    }

    agent::IValuelessVariable const *effect_variable =
            effect_entity->get_variable_parameter(effect->get_full_name());

    temporal::Time time_window_start = cause->get_time();
    temporal::Time effect_agent_time_window_start =
            std::max(effect_entity->get_min_temporal_limit(), time_window_start);
    temporal::Time time_window_end = effect_entity->get_max_temporal_limit();


    structures::ISet<std::string> *relevant_agent_names = new structures::stl::STLSet<std::string>;
    relevant_agent_names->insert(cause->get_entity_name());
    relevant_agent_names->insert(effect->get_entity_name());


    temporal::Time current_time;
    agent::IReadOnlySceneState const *current_scene_state;
    agent::IReadOnlyEntityState const *current_entity_state;
    FP_DATA_TYPE current_reward;
    if (original_scene)
    {
        for (temporal::Time current_time = effect_agent_time_window_start;
             current_time <= time_window_end;
             current_time += scene->get_time_step())
        {
            current_scene_state = scene->get_state(current_time);
            current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
            current_reward = reward_calculator->calculate_state_reward(current_entity_state);
            effect_reward_minimum = std::min(current_reward, effect_reward_minimum);
            delete current_scene_state;
        }
    }
    else
    {
        agent::IScene *scene_copy = scene->scene_deep_copy();
        agent::ISimulationScene *simulation_scene =
                simulation_scene_factory->create_simulation_scene(scene_copy, simulator,
                                                                  scene->get_time_step(),
                                                                  time_window_start,
                                                                  time_window_end,
                                                                  relevant_agent_names);

        for (temporal::Time current_time = effect_agent_time_window_start;
             current_time <= time_window_end;
             current_time += scene->get_time_step())
        {
            current_scene_state = simulation_scene->get_state(current_time);
            current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
            current_reward = reward_calculator->calculate_state_reward(current_entity_state);
            effect_reward_minimum = std::min(current_reward, effect_reward_minimum);
            delete current_scene_state;
        }

        delete simulation_scene;
        delete scene_copy;
    }

    size_t effect_better_action_count = 0;

    for (size_t i = 0; i < branch_count; ++i)
    {
        agent::IScene *branch_scene = scene->scene_deep_copy();

        agent::IEntity *branch_entity =
                branch_scene->get_mutable_entity(effect->get_entity_name());

        agent::IValuelessVariable *branch_valueless_variable =
                branch_entity->get_mutable_variable_parameter(effect->get_full_name());

        agent::IVariable<agent::Goal<FP_DATA_TYPE>> *branch_variable =
                dynamic_cast<agent::IVariable<agent::Goal<FP_DATA_TYPE>>*>(branch_valueless_variable);

        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *alternative_action =
                (*alternative_actions)[i];

        branch_variable->remove_value(effect->get_time());

        branch_variable->set_value(alternative_action->get_time(),
                                     alternative_action->get_value());

        branch_variable->propogate_events_forward(time_window_end);

        if (original_scene)
        {
            time_window_start = alternative_action->get_time();
        }

        agent::ISimulationScene *simulation_scene =
                simulation_scene_factory->create_simulation_scene(branch_scene, simulator,
                                                                  branch_scene->get_time_step(),
                                                                  time_window_start,
                                                                  time_window_end,
                                                                  relevant_agent_names);

        FP_DATA_TYPE branch_reward_minimum = 1.0f;

        for (temporal::Time current_time = effect_agent_time_window_start;
             current_time <= time_window_end;
             current_time += scene->get_time_step())
        {
            current_scene_state = simulation_scene->get_state(current_time);
            current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
            current_reward = reward_calculator->calculate_state_reward(current_entity_state);
            branch_reward_minimum = std::min(current_reward, branch_reward_minimum);
            delete current_scene_state;
        }

        if (effect_reward_minimum >= branch_reward_minimum)
        {
            ++effect_better_action_count;
        }

        delete simulation_scene;

        delete branch_scene;
    }

    delete relevant_agent_names;

    return FP_DATA_TYPE(effect_better_action_count) / FP_DATA_TYPE(branch_count);
}

NecessaryFPGoalCausalLinkTester::NecessaryFPGoalCausalLinkTester(
        agent::IActionSampler<FP_DATA_TYPE> const *action_sampler,
        agent::ISimulationSceneFactory const *simulation_scene_factory,
        agent::ISimulator const *simulator, agent::IRewardCalculator const *reward_calculator,
        FP_DATA_TYPE ate_threshold, size_t branch_count)
    : action_sampler(action_sampler), simulation_scene_factory(simulation_scene_factory),
      simulator(simulator), reward_calculator(reward_calculator), ate_threshold(ate_threshold),
      branch_count(branch_count) {}

bool NecessaryFPGoalCausalLinkTester::test_causal_link(
        agent::IScene const *scene,
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *cause,
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *effect) const
{
    if (cause->get_time() >= effect->get_time())
    {
        throw std::invalid_argument("Potential cause event cannot be at the same time as or after "
                                    "potential effect event");
    }


    agent::IScene *copied_scene = scene->scene_deep_copy();

    agent::IEntity *copied_entity =
            copied_scene->get_mutable_entity(cause->get_entity_name());

    agent::IValuelessVariable *copied_variable =
            copied_entity->get_mutable_variable_parameter(cause->get_full_name());
    copied_variable->propogate_events_forward(scene->get_max_temporal_limit());


    agent::IScene *intervened_scene = scene->scene_deep_copy();

    agent::IEntity *intervened_entity =
            intervened_scene->get_mutable_entity(cause->get_entity_name());

    if (cause->get_time() == intervened_entity->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential cause event cannot be an initialising event");
    }

    agent::IValuelessVariable *intervened_variable =
            intervened_entity->get_mutable_variable_parameter(cause->get_full_name());
    intervened_variable->remove_value(cause->get_time());
    intervened_variable->propogate_events_forward(scene->get_max_temporal_limit());


    agent::IEntity const *effect_entity = scene->get_entity(effect->get_entity_name());

    agent::IValuelessVariable const *effect_variable =
            effect_entity->get_variable_parameter(effect->get_full_name());

    temporal::Time sampling_time_window_start = cause->get_time();
    temporal::Time sampling_time_window_end = effect_entity->get_max_temporal_limit();

    structures::IArray<agent::IValuelessEvent const*> *effect_variable_events =
            effect_variable->get_valueless_events();
    temporal::Time event_time;
    size_t i;
    for (i = 0; i < effect_variable_events->count(); ++i)
    {
        event_time = (*effect_variable_events)[i]->get_time();
        if (event_time > effect->get_time())
        {
            sampling_time_window_end = event_time;
            break;
        }
        else if (event_time < effect->get_time())
        {
            sampling_time_window_start = std::max(event_time, sampling_time_window_start);
        }
    }
    delete effect_variable_events;


    structures::IArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> *alternative_actions =
            new structures::stl::STLStackArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*>(branch_count);

    for (i = 0; i < branch_count; ++i)
    {
        FP_DATA_TYPE new_goal_value;
        temporal::Time new_action_start_time;
        temporal::Time new_action_end_time;

        action_sampler->sample_action(sampling_time_window_start,
                                      sampling_time_window_end,
                                      MIN_ALIGNED_LINEAR_VELOCITY,
                                      MAX_ALIGNED_LINEAR_VELOCITY,
                                      new_goal_value,
                                      new_action_start_time,
                                      new_action_end_time);

        (*alternative_actions)[i] = new agent::BasicEvent(effect_entity->get_name(),
                                                          effect_variable->get_parameter_name(),
                                                          agent::Goal(new_goal_value,
                                                                      new_action_end_time),
                                                          new_action_start_time);
    }


    FP_DATA_TYPE expected_original_reward = this->calculate_expected_reward(copied_scene, cause,
                                                                            effect,
                                                                            alternative_actions,
                                                                            true);
    FP_DATA_TYPE expected_intervened_reward = this->calculate_expected_reward(intervened_scene,
                                                                              cause, effect,
                                                                              alternative_actions,
                                                                              false);

    for (i = 0; i < branch_count; ++i)
    {
        delete (*alternative_actions)[i];
    }
    delete alternative_actions;

    delete copied_scene;
    delete intervened_scene;

    return (expected_original_reward - expected_intervened_reward) >= ate_threshold;
}

}
}
}

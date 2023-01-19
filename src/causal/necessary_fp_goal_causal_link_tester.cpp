
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/agent/defines.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/basic_event.hpp>
#include <ori/simcars/causal/necessary_fp_goal_causal_link_tester.hpp>

#ifdef CD_DEBUG_PRINT
#include <iostream>
#include <iomanip>
#endif

namespace ori
{
namespace simcars
{
namespace causal
{

NecessaryFPGoalCausalLinkTester::NecessaryFPGoalCausalLinkTester(
        agent::IActionSampler<FP_DATA_TYPE> const *action_sampler,
        agent::ISimulationSceneFactory const *simulation_scene_factory,
        agent::ISimulator const *simulator, agent::IRewardCalculator const *reward_calculator,
        FP_DATA_TYPE reward_diff_threshold)
    : action_sampler(action_sampler), simulation_scene_factory(simulation_scene_factory),
      simulator(simulator), reward_calculator(reward_calculator),
      reward_diff_threshold(reward_diff_threshold) {}

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


    agent::IScene *original_scene = scene->scene_deep_copy();
    agent::IEntity *effect_entity = original_scene->get_mutable_entity(effect->get_entity_name());
    agent::IValuelessVariable *effect_variable =
            effect_entity->get_mutable_variable_parameter(effect->get_full_name());
    structures::IArray<agent::IValuelessEvent*> *effect_events =
            effect_variable->get_mutable_valueless_events();

    temporal::Time time_window_start = cause->get_time();
    temporal::Time time_window_end = effect_entity->get_max_temporal_limit();

    size_t i;
    for (i = 0; i < effect_events->count(); ++i)
    {
        agent::IValuelessEvent *effect_event = (*effect_events)[i];
        if (effect_event->get_time() > effect->get_time())
        {
            time_window_end = effect_event->get_time();
            break;
        }
    }

    agent::IEntity *cause_entity = original_scene->get_mutable_entity(cause->get_entity_name());
    agent::IValuelessVariable *cause_variable =
            cause_entity->get_mutable_variable_parameter(cause->get_full_name());
    cause_variable->propogate_events_forward(scene->get_max_temporal_limit());


    structures::ISet<std::string> *relevant_agent_names = new structures::stl::STLSet<std::string>;
    relevant_agent_names->insert(cause->get_entity_name());
    relevant_agent_names->insert(effect->get_entity_name());


    agent::IScene *cause_intervened_scene = original_scene->scene_deep_copy();
    agent::IEntity *cause_intervened_entity =
            cause_intervened_scene->get_mutable_entity(cause->get_entity_name());
    if (cause->get_time() == cause_intervened_entity->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential cause event cannot be an initialising event");
    }
    agent::IValuelessVariable *cause_intervened_variable =
            cause_intervened_entity->get_mutable_variable_parameter(cause->get_full_name());
    cause_intervened_variable->remove_value(cause->get_time());
    cause_intervened_variable->propogate_events_forward(scene->get_max_temporal_limit());
    agent::IScene *simulated_cause_intervened_scene =
            simulation_scene_factory->create_simulation_scene(cause_intervened_scene, simulator,
                                                              scene->get_time_step(),
                                                              time_window_start,
                                                              time_window_end,
                                                              relevant_agent_names);

    agent::IScene *effect_intervened_scene = original_scene->scene_deep_copy();
    agent::IEntity *effect_intervened_entity =
            effect_intervened_scene->get_mutable_entity(effect->get_entity_name());
    if (effect->get_time() == effect_intervened_entity->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential effect event cannot be an initialising event");
    }
    agent::IValuelessVariable *effect_intervened_variable =
            effect_intervened_entity->get_mutable_variable_parameter(effect->get_full_name());
    effect_intervened_variable->remove_value(effect->get_time());
    effect_intervened_variable->propogate_events_forward(scene->get_max_temporal_limit());
    agent::IScene *simulated_effect_intervened_scene =
            simulation_scene_factory->create_simulation_scene(effect_intervened_scene, simulator,
                                                              scene->get_time_step(),
                                                              time_window_start,
                                                              time_window_end,
                                                              relevant_agent_names);

    agent::IScene *cause_effect_intervened_scene = cause_intervened_scene->scene_deep_copy();
    agent::IEntity *cause_effect_intervened_entity =
            cause_effect_intervened_scene->get_mutable_entity(effect->get_entity_name());
    agent::IValuelessVariable *cause_effect_intervened_variable =
            cause_effect_intervened_entity->get_mutable_variable_parameter(effect->get_full_name());
    cause_effect_intervened_variable->remove_value(effect->get_time());
    cause_effect_intervened_variable->propogate_events_forward(scene->get_max_temporal_limit());
    agent::IScene *simulated_cause_effect_intervened_scene =
            simulation_scene_factory->create_simulation_scene(cause_effect_intervened_scene, simulator,
                                                              scene->get_time_step(),
                                                              time_window_start,
                                                              time_window_end,
                                                              relevant_agent_names);

    delete relevant_agent_names;


    FP_DATA_TYPE original_reward_minimum = 1.0f;
    FP_DATA_TYPE cause_intervened_reward_minimum = 1.0f;
    FP_DATA_TYPE effect_intervened_reward_minimum = 1.0f;
    FP_DATA_TYPE cause_effect_intervened_reward_minimum = 1.0f;

    temporal::Time current_time;
    agent::IReadOnlySceneState const *current_scene_state;
    agent::IReadOnlyEntityState const *current_entity_state;
    FP_DATA_TYPE current_reward;
    for (temporal::Time current_time = effect->get_time();
         current_time <= time_window_end;
         current_time += scene->get_time_step())
    {
        current_scene_state = original_scene->get_state(current_time);
        current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_reward = reward_calculator->calculate_state_reward(current_entity_state);
        original_reward_minimum = std::min(current_reward, original_reward_minimum);
        delete current_scene_state;

        current_scene_state = simulated_cause_intervened_scene->get_state(current_time);
        current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_reward = reward_calculator->calculate_state_reward(current_entity_state);
        cause_intervened_reward_minimum = std::min(current_reward, cause_intervened_reward_minimum);
        delete current_scene_state;

        current_scene_state = simulated_effect_intervened_scene->get_state(current_time);
        current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_reward = reward_calculator->calculate_state_reward(current_entity_state);
        effect_intervened_reward_minimum = std::min(current_reward,
                                                    effect_intervened_reward_minimum);
        delete current_scene_state;

        current_scene_state = simulated_cause_effect_intervened_scene->get_state(current_time);
        current_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_reward = reward_calculator->calculate_state_reward(current_entity_state);
        cause_effect_intervened_reward_minimum = std::min(current_reward,
                                                          cause_effect_intervened_reward_minimum);
        delete current_scene_state;
    }


#ifdef CD_DEBUG_PRINT
    std::cout << "┌─────┬─────┬─────┐" << std::endl;
    std::cout << "│     │  E  │ ¬ E │" << std::endl;
    std::cout << "├─────┼─────┼─────┤" << std::endl;
    std::cout << "│  C  │" << std::setprecision(3) << std::fixed << original_reward_minimum << "│" <<
                 std::setprecision(3) << std::fixed << effect_intervened_reward_minimum << "│" <<
                 std::endl;
    std::cout << "├─────┼─────┼─────┤" << std::endl;
    std::cout << "│ ¬ C │" << std::setprecision(3) << std::fixed << cause_intervened_reward_minimum <<
                 "│" << std::setprecision(3) << std::fixed <<
                 cause_effect_intervened_reward_minimum << "│" << std::endl;
    std::cout << "└─────┴─────┴─────┘" << std::endl;
#endif


    FP_DATA_TYPE direct_causal_implication = original_reward_minimum -
            effect_intervened_reward_minimum;
    FP_DATA_TYPE reverse_causal_implication = cause_effect_intervened_reward_minimum -
            cause_intervened_reward_minimum;
    FP_DATA_TYPE combined_causal_implication = direct_causal_implication +
            reverse_causal_implication;
    bool causally_significant = combined_causal_implication >= reward_diff_threshold;


#ifdef CD_DEBUG_PRINT
    std::cout << "Direct Causal Implication = " << direct_causal_implication << std::endl;
    std::cout << "Reverse Causal Implication = " << reverse_causal_implication << std::endl;
    std::cout << "Combined Causal Implication = " << combined_causal_implication << std::endl;
    std::cout << "Causally Significant: " << (causally_significant ? "Yes" : "No") << std::endl;
#endif


    FP_DATA_TYPE effect_entity_impotus = original_reward_minimum -
            cause_intervened_reward_minimum;
    FP_DATA_TYPE cause_entity_impotus = cause_effect_intervened_reward_minimum -
            effect_intervened_reward_minimum;
    bool facilitation_type = effect_entity_impotus >= reward_diff_threshold &&
            cause_entity_impotus <= -reward_diff_threshold;


#ifdef CD_DEBUG_PRINT
    std::cout << "Effect Entity Impotus = " << effect_entity_impotus << std::endl;
    std::cout << "Cause Entity Impotus = " << cause_entity_impotus << std::endl;
    std::cout << "Facilitation Type: " << (facilitation_type ? "Yes" : "No") << std::endl;
#endif


    delete simulated_cause_effect_intervened_scene;
    delete cause_effect_intervened_scene;
    delete simulated_effect_intervened_scene;
    delete effect_intervened_scene;
    delete simulated_cause_intervened_scene;
    delete cause_intervened_scene;
    delete original_scene;


    return causally_significant && !facilitation_type;
}

}
}
}

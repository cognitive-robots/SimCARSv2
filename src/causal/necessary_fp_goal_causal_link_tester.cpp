
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
        agent::IAgencyCalculator const *agency_calculator, FP_DATA_TYPE reward_diff_threshold,
        temporal::Duration simulation_horizon)
    : action_sampler(action_sampler), simulation_scene_factory(simulation_scene_factory),
      simulator(simulator), reward_calculator(reward_calculator),
      agency_calculator(agency_calculator), reward_diff_threshold(reward_diff_threshold),
      simulation_horizon(simulation_horizon) {}

void NecessaryFPGoalCausalLinkTester::test_causal_link(
        agent::IScene const *scene,
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *cause,
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *effect, bool &reward_found,
        bool &agency_found, bool &hybrid_found) const
{
    if (cause->get_time() >= effect->get_time())
    {
        throw std::invalid_argument("Potential cause event cannot be at the same time as or after "
                                    "potential effect event");
    }


    agent::IScene *original_scene = scene->scene_deep_copy();

    agent::IEntity *cause_entity = original_scene->get_mutable_entity(cause->get_entity_name());
    agent::IValuelessVariable *cause_variable =
            cause_entity->get_mutable_variable_parameter(cause->get_full_name());
    structures::IArray<agent::IValuelessEvent*> *cause_events =
            cause_variable->get_mutable_valueless_events();

    agent::IEntity *effect_entity = original_scene->get_mutable_entity(effect->get_entity_name());
    agent::IValuelessVariable *effect_variable =
            effect_entity->get_mutable_variable_parameter(effect->get_full_name());
    structures::IArray<agent::IValuelessEvent*> *effect_events =
            effect_variable->get_mutable_valueless_events();

    temporal::Time time_window_start = cause->get_time();
    temporal::Time time_window_end =
            std::min(effect_entity->get_max_temporal_limit() + simulation_horizon,
                     scene->get_max_temporal_limit());

    cause_variable->propogate_events_forward(time_window_end);
    effect_variable->propogate_events_forward(time_window_end);


    structures::ISet<std::string> *relevant_agent_names = new structures::stl::STLSet<std::string>;
    relevant_agent_names->insert(cause->get_entity_name());
    relevant_agent_names->insert(effect->get_entity_name());


    agent::IScene *simulated_original_scene =
            simulation_scene_factory->create_simulation_scene(
                original_scene, simulator, scene->get_time_step(),
                std::min(cause_entity->get_max_temporal_limit(),
                         effect_entity->get_max_temporal_limit()),
                time_window_end, relevant_agent_names);

    agent::IScene *cause_intervened_scene = original_scene->scene_deep_copy();
    agent::IEntity *cause_intervened_entity =
            cause_intervened_scene->get_mutable_entity(cause->get_entity_name());
    agent::IValuelessVariable *cause_intervened_variable =
            cause_intervened_entity->get_mutable_variable_parameter(cause->get_full_name());
    if (cause->get_time() == cause_intervened_variable->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential cause event cannot be an initialising event");
    }
    cause_intervened_variable->remove_value(cause->get_time());
    cause_intervened_variable->propogate_events_forward(time_window_end);
    agent::IScene *simulated_cause_intervened_scene =
            simulation_scene_factory->create_simulation_scene(cause_intervened_scene, simulator,
                                                              scene->get_time_step(),
                                                              time_window_start, time_window_end,
                                                              relevant_agent_names);


    FP_DATA_TYPE preeffect_min_original_effect_reward = 1.0f;
    FP_DATA_TYPE preeffect_min_cause_intervened_effect_reward = 1.0f;

    bool preeffect_original_cause_agency = true;
    bool preeffect_cause_intervened_cause_agency = true;

    bool preeffect_original_effect_agency = true;
    bool preeffect_cause_intervened_effect_agency = true;

    bool preeffect_original_linked_agency_loss = false;
    bool preeffect_cause_intervened_linked_agency_loss = false;

    FP_DATA_TYPE current_original_effect_reward;
    FP_DATA_TYPE current_cause_intervened_effect_reward;

    bool current_original_cause_agency = true;
    bool current_cause_intervened_cause_agency = true;

    bool current_original_effect_agency = true;
    bool current_cause_intervened_effect_agency = true;

    temporal::Time current_time;
    agent::IReadOnlySceneState const *current_scene_state;
    agent::IReadOnlyEntityState const *current_effect_entity_state;
    agent::IReadOnlyEntityState const *current_cause_entity_state;
    for (temporal::Time current_time = std::max(cause->get_time(),
                                                effect_entity->get_min_temporal_limit());
         current_time <= effect->get_time(); current_time += scene->get_time_step())
    {
        current_scene_state = simulated_original_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(
                    effect->get_entity_name());
        current_original_effect_reward = reward_calculator->calculate_state_reward(
                    current_effect_entity_state);
        preeffect_min_original_effect_reward = std::min(current_original_effect_reward,
                                                 preeffect_min_original_effect_reward);
        current_original_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_original_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        current_scene_state = simulated_cause_intervened_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(
                    effect->get_entity_name());
        current_cause_intervened_effect_reward =
                reward_calculator->calculate_state_reward(current_effect_entity_state);
        preeffect_min_cause_intervened_effect_reward =
                std::min(current_cause_intervened_effect_reward,
                         preeffect_min_cause_intervened_effect_reward);
        current_cause_intervened_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_cause_intervened_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        preeffect_original_linked_agency_loss = preeffect_original_linked_agency_loss ||
                ((preeffect_original_effect_agency && preeffect_original_cause_agency) &&
                 (!current_original_effect_agency && !current_original_cause_agency));
        preeffect_cause_intervened_linked_agency_loss =
                preeffect_cause_intervened_linked_agency_loss ||
                ((preeffect_cause_intervened_effect_agency &&
                  preeffect_cause_intervened_cause_agency) &&
                 (!current_cause_intervened_effect_agency &&
                  !current_cause_intervened_cause_agency));

        preeffect_original_cause_agency = preeffect_original_cause_agency &&
                current_original_cause_agency;
        preeffect_cause_intervened_cause_agency = preeffect_cause_intervened_cause_agency &&
                current_cause_intervened_cause_agency;

        preeffect_original_effect_agency = preeffect_original_effect_agency &&
                current_original_effect_agency;
        preeffect_cause_intervened_effect_agency = preeffect_cause_intervened_effect_agency &&
                current_cause_intervened_effect_agency;
    }


    agent::IScene *effect_intervened_scene = original_scene->scene_deep_copy();
    agent::IEntity *effect_intervened_entity =
            effect_intervened_scene->get_mutable_entity(effect->get_entity_name());
    agent::IValuelessVariable *effect_intervened_variable =
            effect_intervened_entity->get_mutable_variable_parameter(effect->get_full_name());
    if (effect->get_time() == effect_intervened_variable->get_min_temporal_limit())
    {
        throw std::invalid_argument("Potential effect event cannot be an initialising event");
    }
    effect_intervened_variable->remove_value(effect->get_time());
    effect_intervened_variable->propogate_events_forward(time_window_end);
    agent::IScene *simulated_effect_intervened_scene =
            simulation_scene_factory->create_simulation_scene(
                effect_intervened_scene, simulator, scene->get_time_step(),
                std::min(effect->get_time(), cause_entity->get_max_temporal_limit()),
                time_window_end, relevant_agent_names);

    agent::IScene *cause_effect_intervened_scene =
            cause_intervened_scene->scene_deep_copy();
    agent::IEntity *cause_effect_intervened_entity =
            cause_effect_intervened_scene->get_mutable_entity(effect->get_entity_name());
    agent::IValuelessVariable *cause_effect_intervened_variable =
            cause_effect_intervened_entity->get_mutable_variable_parameter(effect->get_full_name());
    cause_effect_intervened_variable->remove_value(effect->get_time());
    cause_effect_intervened_variable->propogate_events_forward(time_window_end);
    agent::IScene *simulated_cause_effect_intervened_scene =
            simulation_scene_factory->create_simulation_scene(cause_effect_intervened_scene,
                                                              simulator, scene->get_time_step(),
                                                              time_window_start, time_window_end,
                                                              relevant_agent_names);

    delete relevant_agent_names;


    FP_DATA_TYPE posteffect_min_original_effect_reward = 1.0f;
    FP_DATA_TYPE posteffect_min_cause_intervened_effect_reward = 1.0f;
    FP_DATA_TYPE posteffect_min_effect_intervened_effect_reward = 1.0f;
    FP_DATA_TYPE posteffect_min_cause_effect_intervened_effect_reward = 1.0f;

    bool posteffect_original_cause_agency = true;
    bool posteffect_cause_intervened_cause_agency = true;
    bool posteffect_effect_intervened_cause_agency = true;
    bool posteffect_cause_effect_intervened_cause_agency = true;

    bool posteffect_original_effect_agency = true;
    bool posteffect_cause_intervened_effect_agency = true;
    bool posteffect_effect_intervened_effect_agency = true;
    bool posteffect_cause_effect_intervened_effect_agency = true;

    bool posteffect_original_linked_agency_loss = false;
    bool posteffect_cause_intervened_linked_agency_loss = false;
    bool posteffect_effect_intervened_linked_agency_loss = false;
    bool posteffect_cause_effect_intervened_linked_agency_loss = false;

    FP_DATA_TYPE current_effect_intervened_effect_reward;
    FP_DATA_TYPE current_cause_effect_intervened_effect_reward;

    bool current_effect_intervened_cause_agency = true;
    bool current_cause_effect_intervened_cause_agency = true;

    bool current_effect_intervened_effect_agency = true;
    bool current_cause_effect_intervened_effect_agency = true;

    for (temporal::Time current_time = effect->get_time() + scene->get_time_step();
         current_time <= time_window_end; current_time += scene->get_time_step())
    {
        current_scene_state = simulated_original_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_original_effect_reward = reward_calculator->calculate_state_reward(current_effect_entity_state);
        posteffect_min_original_effect_reward = std::min(current_original_effect_reward,
                                                  posteffect_min_original_effect_reward);
        current_original_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_original_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        current_scene_state = simulated_cause_intervened_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_cause_intervened_effect_reward =
                reward_calculator->calculate_state_reward(current_effect_entity_state);
        posteffect_min_cause_intervened_effect_reward = std::min(current_cause_intervened_effect_reward,
                                                          posteffect_min_cause_intervened_effect_reward);
        current_cause_intervened_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_cause_intervened_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        current_scene_state = simulated_effect_intervened_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_effect_intervened_effect_reward =
                reward_calculator->calculate_state_reward(current_effect_entity_state);
        posteffect_min_effect_intervened_effect_reward = std::min(current_effect_intervened_effect_reward,
                                                           posteffect_min_effect_intervened_effect_reward);
        current_effect_intervened_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_effect_intervened_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        current_scene_state = simulated_cause_effect_intervened_scene->get_state(current_time);
        current_effect_entity_state = current_scene_state->get_entity_state(effect->get_entity_name());
        current_cause_effect_intervened_effect_reward =
                reward_calculator->calculate_state_reward(current_effect_entity_state);
        posteffect_min_cause_effect_intervened_effect_reward =
                std::min(current_cause_effect_intervened_effect_reward,
                         posteffect_min_cause_effect_intervened_effect_reward);
        current_cause_effect_intervened_effect_agency = agency_calculator->calculate_state_agency(
                    current_effect_entity_state);
        current_cause_entity_state = current_scene_state->get_entity_state(
                    cause->get_entity_name());
        current_cause_effect_intervened_cause_agency = agency_calculator->calculate_state_agency(
                    current_cause_entity_state);
        delete current_scene_state;

        posteffect_original_linked_agency_loss = posteffect_original_linked_agency_loss ||
                ((posteffect_original_effect_agency && posteffect_original_cause_agency) &&
                 (!current_original_effect_agency && !current_original_cause_agency));
        posteffect_cause_intervened_linked_agency_loss =
                posteffect_cause_intervened_linked_agency_loss ||
                ((posteffect_cause_intervened_effect_agency &&
                  posteffect_cause_intervened_cause_agency) &&
                 (!current_cause_intervened_effect_agency &&
                  !current_cause_intervened_cause_agency));
        posteffect_effect_intervened_linked_agency_loss =
                posteffect_effect_intervened_linked_agency_loss ||
                ((posteffect_effect_intervened_effect_agency &&
                  posteffect_effect_intervened_cause_agency) &&
                 (!current_effect_intervened_effect_agency &&
                  !current_effect_intervened_cause_agency));
        posteffect_cause_effect_intervened_linked_agency_loss =
                posteffect_cause_effect_intervened_linked_agency_loss ||
                ((posteffect_cause_effect_intervened_effect_agency &&
                  posteffect_cause_effect_intervened_cause_agency) &&
                 (!current_cause_effect_intervened_effect_agency &&
                  !current_cause_effect_intervened_cause_agency));

        posteffect_original_cause_agency = posteffect_original_cause_agency &&
                current_original_cause_agency;
        posteffect_cause_intervened_cause_agency = posteffect_cause_intervened_cause_agency &&
                current_cause_intervened_cause_agency;
        posteffect_effect_intervened_cause_agency = posteffect_effect_intervened_cause_agency &&
                current_effect_intervened_cause_agency;
        posteffect_cause_effect_intervened_cause_agency = posteffect_cause_effect_intervened_cause_agency &&
                current_cause_effect_intervened_cause_agency;

        posteffect_original_effect_agency = posteffect_original_effect_agency &&
                current_original_effect_agency;
        posteffect_cause_intervened_effect_agency = posteffect_cause_intervened_effect_agency &&
                current_cause_intervened_effect_agency;
        posteffect_effect_intervened_effect_agency = posteffect_effect_intervened_effect_agency &&
                current_effect_intervened_effect_agency;
        posteffect_cause_effect_intervened_effect_agency = posteffect_cause_effect_intervened_effect_agency &&
                current_cause_effect_intervened_effect_agency;
    }


#ifdef CD_DEBUG_PRINT
    std::cout << "┌─────┬─────┬─────┬─────┐" << std::endl;
    std::cout << "│ Re. │  E  │ ¬ E │ PRE │" << std::endl;
    std::cout << "├─────┼─────┼─────┬─────┤" << std::endl;
    std::cout << "│  C  │" << std::setprecision(3) << std::fixed <<
                 posteffect_min_original_effect_reward << "│" << std::setprecision(3) << std::fixed <<
                 posteffect_min_effect_intervened_effect_reward << "│" << std::setprecision(3) <<
                 std::fixed << preeffect_min_original_effect_reward << "│" << std::endl;
    std::cout << "├─────┼─────┼─────┼─────┤" << std::endl;
    std::cout << "│ ¬ C │" << std::setprecision(3) << std::fixed <<
                 posteffect_min_cause_intervened_effect_reward << "│" << std::setprecision(3) <<
                 std::fixed << posteffect_min_cause_effect_intervened_effect_reward << "│" <<
                 std::setprecision(3) << std::fixed <<
                 preeffect_min_cause_intervened_effect_reward << "│" << std::endl;
    std::cout << "└─────┴─────┴─────┴─────┘" << std::endl;
    std::cout << "┌─────┬─────┬─────┬─────┐" << std::endl;
    std::cout << "│ Ag. │  E  │ ¬ E │ PRE │" << std::endl;
    std::cout << "├─────┼─────┼─────┬─────┤" << std::endl;
    std::cout << "│  C  │" << std::setw(5) << !posteffect_original_linked_agency_loss << "│" <<
                 std::setw(5) << !posteffect_effect_intervened_linked_agency_loss << "│" <<
                 std::setw(5) << !preeffect_original_linked_agency_loss << "│" << std::endl;
    std::cout << "├─────┼─────┼─────┼─────┤" << std::endl;
    std::cout << "│ ¬ C │" << std::setw(5) << !posteffect_cause_intervened_linked_agency_loss <<
                 "│" << std::setw(5) << !posteffect_cause_effect_intervened_linked_agency_loss <<
                 "│" << std::setw(5) << !preeffect_cause_intervened_linked_agency_loss << "│" <<
                 std::endl;
    std::cout << "└─────┴─────┴─────┴─────┘" << std::endl;
#endif

    FP_DATA_TYPE direct_causal_implication = posteffect_min_original_effect_reward -
            posteffect_min_effect_intervened_effect_reward;
    FP_DATA_TYPE reverse_causal_implication = posteffect_min_cause_effect_intervened_effect_reward -
            posteffect_min_cause_intervened_effect_reward;
    FP_DATA_TYPE combined_causal_implication = direct_causal_implication +
            reverse_causal_implication;
    bool causally_significant = combined_causal_implication >= reward_diff_threshold;


#ifdef CD_DEBUG_PRINT
    std::cout << "Direct Causal Implication = " << direct_causal_implication << std::endl;
    std::cout << "Reverse Causal Implication = " << reverse_causal_implication << std::endl;
    std::cout << "Combined Causal Implication = " << combined_causal_implication << std::endl;
    std::cout << "Causally Significant: " << (causally_significant ? "Yes" : "No") << std::endl;
#endif


    bool active_type = !posteffect_original_linked_agency_loss &&
            posteffect_effect_intervened_linked_agency_loss &&
            !posteffect_cause_effect_intervened_linked_agency_loss;
    bool passive_type = !posteffect_original_linked_agency_loss &&
            posteffect_cause_intervened_linked_agency_loss &&
            !posteffect_cause_effect_intervened_linked_agency_loss;
    bool facilitation_type = !posteffect_original_linked_agency_loss &&
            posteffect_cause_intervened_linked_agency_loss &&
            posteffect_cause_effect_intervened_linked_agency_loss;
    bool mutual_effect_motive = !posteffect_original_linked_agency_loss &&
            !posteffect_cause_intervened_linked_agency_loss &&
            posteffect_effect_intervened_linked_agency_loss &&
            posteffect_cause_effect_intervened_linked_agency_loss;


#ifdef CD_DEBUG_PRINT
    std::cout << "Facilitation Type: " << (facilitation_type ? "Yes" : "No") << std::endl;
#endif


    delete simulated_cause_effect_intervened_scene;
    delete cause_effect_intervened_scene;
    delete simulated_effect_intervened_scene;
    delete effect_intervened_scene;
    delete simulated_cause_intervened_scene;
    delete cause_intervened_scene;
    delete simulated_original_scene;
    delete original_scene;

    reward_found = causally_significant;
    agency_found = (active_type || passive_type) && !(facilitation_type || mutual_effect_motive);
    hybrid_found = (causally_significant || active_type || passive_type) &&
            !(facilitation_type || mutual_effect_motive);
}

}
}
}

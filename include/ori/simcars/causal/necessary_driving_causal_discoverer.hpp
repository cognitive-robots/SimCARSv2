#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/basic_fp_action_sampler.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/driving_simulation_scene_factory.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/safe_speedy_driving_agent_reward_calculator.hpp>
#include <ori/simcars/causal/causal_discoverer_interface.hpp>
#include <ori/simcars/causal/necessary_fp_goal_causal_link_tester.hpp>

#include <thread>

#define GOLDEN_RATIO_MAGIC_NUM 0x9e3779b9

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename T1, typename T2>
class PairHasher
{
    std::hash<T1> hasher_1;
    std::hash<T2> hasher_2;

public:
    std::size_t operator()(std::pair<T1, T2> const &key) const
    {
        size_t key_hash = hasher_1(key.first);
        key_hash ^= hasher_2(key.second) + GOLDEN_RATIO_MAGIC_NUM + (key_hash << 6) + (key_hash >> 2);
        return key_hash;
    }
};


template <typename T_map_id>
class NecessaryDrivingCausalDiscoverer : public virtual ICausalDiscoverer
{
    map::IMap<T_map_id> const *map;

    agent::IActionSampler<FP_DATA_TYPE> const *action_sampler;
    agent::ISimulationSceneFactory const *simulation_scene_factory;
    agent::IDrivingAgentController const *controller;
    agent::IDrivingSimulator const *simulator;
    agent::IRewardCalculator const *reward_calculator;

    ICausalLinkTester<agent::Goal<FP_DATA_TYPE>, agent::Goal<FP_DATA_TYPE>> const *causal_link_tester;

public:
    NecessaryDrivingCausalDiscoverer(map::IMap<T_map_id> const *map, temporal::Duration time_step,
                                     size_t controller_lookahead_steps, FP_DATA_TYPE ate_threshold,
                                     size_t branch_count)
        : map(map), action_sampler(new agent::BasicFPActionSampler),
          simulation_scene_factory(new agent::DrivingSimulationSceneFactory),
          controller(new agent::BasicDrivingAgentController<T_map_id>(map, time_step,
                                                                      controller_lookahead_steps)),
          simulator(new agent::BasicDrivingSimulator(controller)),
          reward_calculator(new agent::SafeSpeedyDrivingAgentRewardCalculator),
          causal_link_tester(new NecessaryFPGoalCausalLinkTester(action_sampler,
                                                                 simulation_scene_factory, simulator,
                                                                 reward_calculator, ate_threshold,
                                                                 branch_count))
    {
    }

    ~NecessaryDrivingCausalDiscoverer() override
    {
        delete causal_link_tester;

        delete reward_calculator;
        delete simulator;
        delete controller;
        delete simulation_scene_factory;
        delete action_sampler;
    }

    structures::ISet<std::pair<std::string, std::string>>* discover_entity_causal_links(
            agent::IScene const *scene,
            structures::ISet<std::string> const *agents_of_interest) const override
    {
        agent::IDrivingScene const *driving_scene =
                dynamic_cast<agent::IDrivingScene const*>(scene);
        if (driving_scene == nullptr)
        {
            throw std::invalid_argument("Scene was not a driving scene");
        }

        agent::IDrivingScene *driving_scene_copy = driving_scene->driving_scene_deep_copy();

        agent::IDrivingScene *driving_scene_with_actions =
                agent::DrivingGoalExtractionScene<T_map_id>::construct_from(driving_scene_copy,
                                                                            map);



        structures::IArray<agent::IDrivingAgent const*> const *driving_agents_with_actions =
                driving_scene_with_actions->get_driving_agents();

        structures::stl::STLStackArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> aligned_linear_velocity_goal_events;

        size_t i, j;
        for (i = 0; i < driving_agents_with_actions->count(); ++i)
        {
            agent::IDrivingAgent const *driving_agent_with_actions =
                    (*driving_agents_with_actions)[i];
            if (agents_of_interest == nullptr ||
                    agents_of_interest->contains(driving_agent_with_actions->get_name()))
            {
                agent::IValuelessVariable const *driving_agent_aligned_linear_velocity_goal_valueless_variable =
                        driving_agent_with_actions->get_variable_parameter(driving_agent_with_actions->get_name() +
                                                                           ".aligned_linear_velocity.goal");

                if (driving_agent_aligned_linear_velocity_goal_valueless_variable == nullptr)
                {
                    throw std::runtime_error("No aligned linear velocity goal variable present on agent of interest");
                }

                agent::IVariable<agent::Goal<FP_DATA_TYPE>> const *driving_agent_aligned_linear_velocity_goal_variable =
                        dynamic_cast<agent::IVariable<agent::Goal<FP_DATA_TYPE>> const*>(
                                driving_agent_aligned_linear_velocity_goal_valueless_variable);

                structures::IArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> const *driving_agent_aligned_linear_velocity_goal_events =
                        driving_agent_aligned_linear_velocity_goal_variable->get_events();

                for (j = 0; j < driving_agent_aligned_linear_velocity_goal_events->count(); ++j)
                {
                    agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *driving_agent_aligned_linear_velocity_goal_event =
                            (*driving_agent_aligned_linear_velocity_goal_events)[j];

                    if (driving_agent_aligned_linear_velocity_goal_event->get_time() >
                            driving_agent_with_actions->get_min_temporal_limit())
                    {
                        aligned_linear_velocity_goal_events.push_back(
                                    driving_agent_aligned_linear_velocity_goal_event);
                    }
                }

                delete driving_agent_aligned_linear_velocity_goal_events;
            }
        }

        delete driving_agents_with_actions;



        structures::stl::STLStackArray<std::thread*> threads(
                    std::pow(aligned_linear_velocity_goal_events.count(), 2), nullptr);
        structures::stl::STLStackArray<std::pair<std::string, std::string>*> discovered_entity_causal_link_array(
                    std::pow(aligned_linear_velocity_goal_events.count(), 2), nullptr);

        for (i = 0; i < aligned_linear_velocity_goal_events.count(); ++i)
        {
            agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_cause =
                    aligned_linear_velocity_goal_events[i];
            for (j = 0; j < aligned_linear_velocity_goal_events.count(); ++j)
            {
                agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_effect =
                        aligned_linear_velocity_goal_events[j];

                if (potential_cause->get_entity_name() != potential_effect->get_entity_name() &&
                        potential_cause->get_time() < potential_effect->get_time())
                {
                    threads[i * aligned_linear_velocity_goal_events.count() + j] =
                            new std::thread([&, i, j, potential_cause, potential_effect]()
                    {
                        bool link_present = causal_link_tester->test_causal_link(
                                    driving_scene_with_actions, potential_cause, potential_effect);

                        if (link_present)
                        {
                            std::string cause_driving_agent = potential_cause->get_entity_name();
                            std::string effect_driving_agent = potential_effect->get_entity_name();
                            std::pair<std::string, std::string> *entity_causal_link =
                                    new std::pair<std::string, std::string>(cause_driving_agent,
                                                                            effect_driving_agent);
                            discovered_entity_causal_link_array[i * aligned_linear_velocity_goal_events.count() + j] =
                                    entity_causal_link;
                        }
                    });
                }
            }
        }

        structures::ISet<std::pair<std::string, std::string>> *discovered_entity_causal_links =
                new structures::stl::STLSet<std::pair<std::string, std::string>, PairHasher<std::string, std::string>>;

        assert(threads.count() == discovered_entity_causal_link_array.count());
        for (i = 0; i < threads.count(); ++i)
        {
            std::thread *thread = threads[i];
            if (thread != nullptr)
            {
                thread->join();
                delete thread;
                if (discovered_entity_causal_link_array[i] != nullptr)
                {
                    discovered_entity_causal_links->insert(*(discovered_entity_causal_link_array[i]));
                    delete discovered_entity_causal_link_array[i];
                }
            }
        }



        delete driving_scene_with_actions;

        delete driving_scene_copy;

        return discovered_entity_causal_links;
    }
};

}
}
}

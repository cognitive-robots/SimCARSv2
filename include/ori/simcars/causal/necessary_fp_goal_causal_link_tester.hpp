#pragma once

#include <ori/simcars/agent/action_sampler_interface.hpp>
#include <ori/simcars/agent/simulation_scene_factory_interface.hpp>
#include <ori/simcars/agent/simulator_interface.hpp>
#include <ori/simcars/agent/reward_calculator_interface.hpp>
#include <ori/simcars/agent/agency_calculator_interface.hpp>
#include <ori/simcars/agent/goal.hpp>
#include <ori/simcars/causal/causal_link_tester_interface.hpp>

//#define CD_DEBUG_PRINT

namespace ori
{
namespace simcars
{
namespace causal
{

class NecessaryFPGoalCausalLinkTester
        : public virtual ICausalLinkTester<agent::Goal<FP_DATA_TYPE>, agent::Goal<FP_DATA_TYPE>>
{
    agent::IActionSampler<FP_DATA_TYPE> const *action_sampler;
    agent::ISimulationSceneFactory const *simulation_scene_factory;
    agent::ISimulator const *simulator;
    agent::IRewardCalculator const *reward_calculator;
    agent::IAgencyCalculator const *agency_calculator;

    FP_DATA_TYPE reward_diff_threshold;
    temporal::Duration simulation_horizon;

public:
    NecessaryFPGoalCausalLinkTester(agent::IActionSampler<FP_DATA_TYPE> const *action_sampler,
                                    agent::ISimulationSceneFactory const *simulation_scene_factory,
                                    agent::ISimulator const *simulator,
                                    agent::IRewardCalculator const *reward_calculator,
                                    agent::IAgencyCalculator const *agency_calculator,
                                    FP_DATA_TYPE reward_diff_threshold,
                                    temporal::Duration simulation_horizon);

    bool test_causal_link(agent::IScene const *scene,
                          agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *cause,
                          agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *effect) const override;
};

}
}
}

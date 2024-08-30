
#include <ori/simcars/agents/default_ped_reward_calc.hpp>

#define TASK_GOAL_VARIANCE 100
#define SPACE_VARIANCE 0.5

namespace ori
{
namespace simcars
{
namespace agents
{

DefaultPedRewardCalc::DefaultPedRewardCalc(map::IPedMap const *map) : map(map) {}

FP_DATA_TYPE DefaultPedRewardCalc::calc_reward(PedOutcome const *outcome, PedTask const *task,
                                               PedRewardParameters const *parameters) const
{
    PedRewards rewards = calc_rewards(outcome, task, parameters);

    return parameters->task_goal_weight * rewards.task_goal_reward +
            parameters->space_weight * rewards.space_reward +
            parameters->bias_weight;
}

PedRewards DefaultPedRewardCalc::calc_rewards(PedOutcome const *outcome, PedTask const *task,
                                              PedRewardParameters const *parameters) const
{
    PedRewards rewards;

    map::INode const *task_node = map->get_node(task->task_node_id);
    switch (task->task_op)
    {
    case PedTask::TaskOp::NOOP:
        rewards.task_goal_reward = 1.0;
        break;
    case PedTask::TaskOp::MOVE:
    {
        FP_DATA_TYPE task_node_squared_dist =
                (task_node->get_centroid() - outcome->pos).squaredNorm();
        rewards.task_goal_reward = std::exp(-0.5 * task_node_squared_dist / TASK_GOAL_VARIANCE);
        break;
    }
    case PedTask::TaskOp::UNKNOWN:
    default:
        throw std::runtime_error("Unrecognised task operation type");
    }

    rewards.space_reward = 1.0 - std::exp(-0.5 * outcome->min_neighbour_dist / SPACE_VARIANCE);

    return rewards;
}

}
}
}

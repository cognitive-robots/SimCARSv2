#pragma once

#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_quotient.hpp>
#include <ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp>
#include <ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp>
#include <ori/simcars/causal/variable_types/endogenous/node_centroid.hpp>
#include <ori/simcars/agents/control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class GoalForceControlPed : public virtual ControlPed
{
protected:
    void init_links() override;

    map::IPedMap const *map;

    simcars::causal::NodeCentroidVariable goal_pos;
    simcars::causal::VectorSocketVariable pos;
    simcars::causal::VectorNegationVariable neg_pos;
    simcars::causal::VectorSumVariable pos_diff;

    simcars::causal::TimeCurrentTimeDifferenceVariable time_diff;
    simcars::causal::DurationSecondsCastVariable time_diff_secs;
    simcars::causal::ScalarFixedVariable min_act_horizon_secs;
    simcars::causal::ScalarMaxVariable actual_act_horizon_secs;
    simcars::causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    simcars::causal::VectorScalarProductVariable goal_lin_vel;
    simcars::causal::VectorSocketVariable lin_vel;
    simcars::causal::VectorNegationVariable neg_lin_vel;
    simcars::causal::VectorSumVariable lin_vel_diff;
    simcars::causal::VectorTimeStepSizeQuotientVariable lin_acc;

    simcars::causal::ScalarSocketVariable mass;
    simcars::causal::VectorScalarProductVariable goal_force;

    simcars::causal::ScalarFixedVariable max_goal_force_mag;
    simcars::causal::VectorNormVariable goal_force_mag;
    simcars::causal::ScalarMinVariable actual_goal_force_mag;

    simcars::causal::VectorNormalisationVariable goal_force_dir;
    simcars::causal::VectorScalarProductVariable actual_goal_force;

public:
    GoalForceControlPed(map::IPedMap const *map, FP_DATA_TYPE max_goal_force_mag_value);
    GoalForceControlPed(GoalForceControlPed const &control_fwd_car);

    simcars::causal::IEndogenousVariable<geometry::Vec>* get_pos_diff_variable();
};

}
}
}

#pragma once

#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/ids_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_encapsulating.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_transitions_calc.hpp>
#include <ori/simcars/agents/full_control_fwd_car.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_time_conditional.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FullControlFWDCarSim : public FullControlFWDCar
{
    FullControlFWDCar const* const original_full_control_fwd_car;

protected:
    causal::FWDCarActionTimeConditionalVariable sim_action;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_diff;

    simcars::causal::ScalarFixedVariable zero_cumil_lane_trans;
    simcars::causal::ScalarProxyVariable zero_cumil_lane_trans_proxy;

    simcars::causal::LaneEncapsulatingVariable lane_encaps;
    simcars::causal::IdsPreviousTimeStepVariable prev_lane_encaps;
    simcars::causal::LaneTransitionsCalcVariable lane_trans;

    simcars::causal::ScalarTimeConditionalVariable sim_cumil_lane_trans;
    simcars::causal::ScalarPreviousTimeStepVariable prev_cumil_lane_trans;

    simcars::causal::ScalarSumVariable cumil_lane_trans;

public:
    FullControlFWDCarSim(FullControlFWDCar *full_control_fwd_car, temporal::Time start_tme);

    simcars::causal::IEndogenousVariable<FWDCarAction>* get_action_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_diff_variable() override;

    simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<uint64_t>>* get_lane_encaps_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_cumil_lane_trans_variable();
};

}
}
}

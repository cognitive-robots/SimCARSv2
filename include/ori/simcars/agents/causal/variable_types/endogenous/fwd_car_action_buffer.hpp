#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarActionBufferVariable :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarAction, FWDCarAction>
{
    temporal::TemporalDictionary<FWDCarAction>* temporal_dictionary;

    bool axiomatic;

public:
    FWDCarActionBufferVariable(
            simcars::causal::IVariable<FWDCarAction> *parent,
            temporal::TemporalDictionary<FWDCarAction> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~FWDCarActionBufferVariable();

    bool get_value(FWDCarAction &val) const override;

    bool set_value(FWDCarAction const &val) override;

    temporal::Time get_min_time() const;
    temporal::Time get_max_time() const;

    void set_axiomatic(bool axiomatic);
};

}
}
}
}

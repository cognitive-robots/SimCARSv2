#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedActionBufferVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedAction, PedAction>
{
    temporal::TemporalDictionary<PedAction>* temporal_dictionary;

    bool axiomatic;

public:
    PedActionBufferVariable(
            simcars::causal::IVariable<PedAction> *parent,
            temporal::TemporalDictionary<PedAction> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~PedActionBufferVariable();

    bool get_value(PedAction &val) const override;

    bool set_value(PedAction const &val) override;

    temporal::Time get_min_time() const;
    temporal::Time get_max_time() const;

    void set_axiomatic(bool axiomatic);
};

}
}
}
}

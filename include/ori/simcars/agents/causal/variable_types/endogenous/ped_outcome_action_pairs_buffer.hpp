#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

bool operator ==(PedOutcomeActionPairs const &outcome_action_pairs_1,
                 PedOutcomeActionPairs const &outcome_action_pairs_2);

namespace causal
{

class PedOutcomeActionPairsBufferVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedOutcomeActionPairs, PedOutcomeActionPairs>
{
    temporal::TemporalDictionary<PedOutcomeActionPairs>* temporal_dictionary;

    bool axiomatic;

public:
    PedOutcomeActionPairsBufferVariable(
            simcars::causal::IVariable<PedOutcomeActionPairs> *parent,
            temporal::TemporalDictionary<PedOutcomeActionPairs> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~PedOutcomeActionPairsBufferVariable();

    bool get_value(PedOutcomeActionPairs &val) const override;

    bool set_value(PedOutcomeActionPairs const &val) override;

    temporal::Time get_min_time() const;
    temporal::Time get_max_time() const;

    void set_axiomatic(bool axiomatic);
};

}
}
}
}

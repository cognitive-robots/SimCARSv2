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

bool operator ==(FWDCarOutcomeActionPairs const &outcome_action_pairs_1,
                 FWDCarOutcomeActionPairs const &outcome_action_pairs_2);

namespace causal
{

class FWDCarOutcomeActionPairsBufferVariable :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarOutcomeActionPairs, FWDCarOutcomeActionPairs>
{
    temporal::TemporalDictionary<FWDCarOutcomeActionPairs>* temporal_dictionary;

    bool axiomatic;

public:
    FWDCarOutcomeActionPairsBufferVariable(
            simcars::causal::IVariable<FWDCarOutcomeActionPairs> *parent,
            temporal::TemporalDictionary<FWDCarOutcomeActionPairs> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~FWDCarOutcomeActionPairsBufferVariable();

    bool get_value(FWDCarOutcomeActionPairs &val) const override;

    bool set_value(FWDCarOutcomeActionPairs const &val) override;

    temporal::Time get_min_time() const;
    temporal::Time get_max_time() const;

    void set_axiomatic(bool axiomatic);
};

}
}
}
}

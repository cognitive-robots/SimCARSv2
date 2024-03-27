
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pairs_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

bool operator ==(FWDCarOutcomeActionPairs const &outcome_action_pairs_1,
                 FWDCarOutcomeActionPairs const &outcome_action_pairs_2)
{
    if (outcome_action_pairs_1.count() != outcome_action_pairs_2.count()) return false;

    for (size_t i = 0; i < outcome_action_pairs_1.count(); ++i)
    {
        if (outcome_action_pairs_1[i] != outcome_action_pairs_2[i]) return false;
    }

    return true;
}

namespace causal
{

FWDCarOutcomeActionPairsBufferVariable::FWDCarOutcomeActionPairsBufferVariable(
        IVariable<FWDCarOutcomeActionPairs> *parent,
        temporal::TemporalDictionary<FWDCarOutcomeActionPairs> *temporal_dictionary,
        bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary =
                new temporal::TemporalDictionary<FWDCarOutcomeActionPairs>;
    }
}

FWDCarOutcomeActionPairsBufferVariable::~FWDCarOutcomeActionPairsBufferVariable()
{
    delete temporal_dictionary;
}

bool FWDCarOutcomeActionPairsBufferVariable::get_value(FWDCarOutcomeActionPairs &val) const
{
    if (temporal_dictionary->contains(simcars::causal::VariableContext::get_current_time(),
                                      simcars::causal::VariableContext::get_time_step_size()))
    {
        val = (*temporal_dictionary)[simcars::causal::VariableContext::get_current_time()];
        return true;
    }
    else if (axiomatic)
    {
        return false;
    }
    else
    {
        if (get_parent()->get_value(val))
        {
            temporal_dictionary->update(simcars::causal::VariableContext::get_current_time(), val);
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool FWDCarOutcomeActionPairsBufferVariable::set_value(FWDCarOutcomeActionPairs const &val)
{
    if (axiomatic && !temporal_dictionary->contains(
                simcars::causal::VariableContext::get_current_time()))
    {
        temporal_dictionary->update(simcars::causal::VariableContext::get_current_time(), val);
        return true;
    }
    else
    {
        if (get_parent()->set_value(val))
        {
            temporal_dictionary->update(simcars::causal::VariableContext::get_current_time(), val);
            return true;
        }
        else
        {
            return false;
        }
    }
}

temporal::Time FWDCarOutcomeActionPairsBufferVariable::get_min_time() const
{
    return temporal_dictionary->get_earliest_time();
}

temporal::Time FWDCarOutcomeActionPairsBufferVariable::get_max_time() const
{
    return temporal_dictionary->get_latest_time();
}

void FWDCarOutcomeActionPairsBufferVariable::set_axiomatic(bool axiomatic)
{
    this->axiomatic = axiomatic;
}

}
}
}
}

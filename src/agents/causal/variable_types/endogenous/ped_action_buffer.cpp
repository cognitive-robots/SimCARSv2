
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedActionBufferVariable::PedActionBufferVariable(
        IVariable<PedAction> *parent,
        temporal::TemporalDictionary<PedAction> *temporal_dictionary, bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary = new temporal::TemporalDictionary<PedAction>;
    }
}

PedActionBufferVariable::~PedActionBufferVariable()
{
    delete temporal_dictionary;
}

bool PedActionBufferVariable::get_value(PedAction &val) const
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

bool PedActionBufferVariable::set_value(PedAction const &val)
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

temporal::Time PedActionBufferVariable::get_min_time() const
{
    return temporal_dictionary->get_earliest_time();
}

temporal::Time PedActionBufferVariable::get_max_time() const
{
    return temporal_dictionary->get_latest_time();
}

void PedActionBufferVariable::set_axiomatic(bool axiomatic)
{
    this->axiomatic = axiomatic;
}

}
}
}
}

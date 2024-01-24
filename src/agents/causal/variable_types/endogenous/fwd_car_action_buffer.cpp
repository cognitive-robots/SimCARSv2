
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarActionBufferVariable::FWDCarActionBufferVariable(
        IVariable<FWDCarAction> *parent,
        temporal::TemporalDictionary<FWDCarAction> *temporal_dictionary, bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary = new temporal::TemporalDictionary<FWDCarAction>;
    }
}

FWDCarActionBufferVariable::~FWDCarActionBufferVariable()
{
    delete temporal_dictionary;
}

bool FWDCarActionBufferVariable::get_value(FWDCarAction &val) const
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

bool FWDCarActionBufferVariable::set_value(FWDCarAction const &val)
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

void FWDCarActionBufferVariable::set_axiomatic(bool axiomatic)
{
    this->axiomatic = axiomatic;
}

}
}
}
}

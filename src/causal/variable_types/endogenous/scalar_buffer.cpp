
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarBufferVariable::ScalarBufferVariable(
        IVariable<FP_DATA_TYPE> *parent,
        temporal::TemporalDictionary<FP_DATA_TYPE> *temporal_dictionary, bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary = new temporal::TemporalDictionary<FP_DATA_TYPE>;
    }
}

ScalarBufferVariable::~ScalarBufferVariable()
{
    delete temporal_dictionary;
}

bool ScalarBufferVariable::get_value(FP_DATA_TYPE &val) const
{
    if (temporal_dictionary->contains(VariableContext::get_current_time()))
    {
        val = (*temporal_dictionary)[VariableContext::get_current_time()];
        return true;
    }
    else if (axiomatic &&
             VariableContext::get_current_time() < temporal_dictionary->get_earliest_time())
    {
        return false;
    }
    else
    {
        if (get_parent()->get_value(val))
        {
            temporal_dictionary->update(VariableContext::get_current_time(), val);
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool ScalarBufferVariable::set_value(FP_DATA_TYPE const &val)
{
    if (temporal_dictionary->contains(VariableContext::get_current_time()))
    {
        if (get_parent()->set_value(val))
        {
            temporal_dictionary->update(VariableContext::get_current_time(), val);
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (axiomatic &&
             VariableContext::get_current_time() < temporal_dictionary->get_earliest_time())
    {
        temporal_dictionary->update(VariableContext::get_current_time(), val);
        return true;
    }
    else
    {
        if (get_parent()->set_value(val))
        {
            temporal_dictionary->update(VariableContext::get_current_time(), val);
            return true;
        }
        else
        {
            return false;
        }
    }
}

}
}
}

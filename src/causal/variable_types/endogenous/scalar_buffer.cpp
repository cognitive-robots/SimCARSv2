
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarBufferVariable::ScalarBufferVariable(
        IVariable<FP_DATA_TYPE> const *parent,
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

FP_DATA_TYPE ScalarBufferVariable::get_value() const
{
    if (temporal_dictionary->contains(VariableContext::get_current_time()))
    {
        return (*temporal_dictionary)[VariableContext::get_current_time()];
    }
    else if (axiomatic &&
             VariableContext::get_current_time() < temporal_dictionary->get_earliest_time())
    {
        return std::numeric_limits<FP_DATA_TYPE>::quiet_NaN();
    }
    else
    {
        FP_DATA_TYPE value = get_parent()->get_value();
        temporal_dictionary->update(VariableContext::get_current_time(), value);
        return value;
    }
}

}
}
}

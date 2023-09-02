
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarBufferVariable::ScalarBufferVariable(IVariable<FP_DATA_TYPE> const *parent) :
    ScalarBufferVariable(parent, new temporal::TemporalRoundingDictionary<FP_DATA_TYPE>(
                             VariableContext::get_time_step_size(),
                             std::numeric_limits<FP_DATA_TYPE>::quiet_NaN())) {}

ScalarBufferVariable::ScalarBufferVariable(
        IVariable<FP_DATA_TYPE> const *parent,
        temporal::TemporalRoundingDictionary<FP_DATA_TYPE> *temporal_dictionary) :
    AUnaryEndogenousVariable(parent), temporal_dictionary(temporal_dictionary)
{
    assert(temporal_dictionary != nullptr);
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

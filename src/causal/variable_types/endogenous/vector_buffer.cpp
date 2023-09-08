
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorBufferVariable::VectorBufferVariable(
        IVariable<geometry::Vec> const *parent,
        temporal::TemporalRoundingDictionary<geometry::Vec> *temporal_dictionary, bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary = new temporal::TemporalRoundingDictionary<geometry::Vec>(
                    VariableContext::get_time_step_size(),
                    geometry::Vec(std::numeric_limits<FP_DATA_TYPE>::quiet_NaN(),
                                  std::numeric_limits<FP_DATA_TYPE>::quiet_NaN()));
    }
}

VectorBufferVariable::~VectorBufferVariable()
{
    delete temporal_dictionary;
}

geometry::Vec VectorBufferVariable::get_value() const
{
    if (temporal_dictionary->contains(VariableContext::get_current_time()))
    {
        return (*temporal_dictionary)[VariableContext::get_current_time()];
    }
    else if (axiomatic &&
             VariableContext::get_current_time() < temporal_dictionary->get_earliest_timestamp())
    {
        return temporal_dictionary->get_default_value();
    }
    else
    {
        geometry::Vec value = get_parent()->get_value();
        temporal_dictionary->update(VariableContext::get_current_time(), value);
        return value;
    }
}

}
}
}

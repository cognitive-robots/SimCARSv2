
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorBufferVariable::VectorBufferVariable(IVariable<geometry::Vec> const *parent) :
    VectorBufferVariable(parent, new temporal::TemporalRoundingDictionary<geometry::Vec>(
                             VariableContext::get_time_step_size(),
                             geometry::Vec(std::numeric_limits<FP_DATA_TYPE>::quiet_NaN(),
                                           std::numeric_limits<FP_DATA_TYPE>::quiet_NaN()))) {}

VectorBufferVariable::VectorBufferVariable(
        IVariable<geometry::Vec> const *parent,
        temporal::TemporalRoundingDictionary<geometry::Vec> *temporal_dictionary) :
    AUnaryEndogenousVariable(parent), temporal_dictionary(temporal_dictionary)
{
    assert(temporal_dictionary != nullptr);
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


#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

VectorBufferVariable::VectorBufferVariable(
        IVariable<geometry::Vec> *parent,
        temporal::TemporalDictionary<geometry::Vec> *temporal_dictionary, bool axiomatic) :
    AUnaryEndogenousVariable(parent), axiomatic(axiomatic)
{
    if (temporal_dictionary != nullptr)
    {
        this->temporal_dictionary = temporal_dictionary;
    }
    else
    {
        this->temporal_dictionary = new temporal::TemporalDictionary<geometry::Vec>;
    }
}

VectorBufferVariable::~VectorBufferVariable()
{
    delete temporal_dictionary;
}

bool VectorBufferVariable::get_value(geometry::Vec &val) const
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

bool VectorBufferVariable::set_value(geometry::Vec const &val)
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

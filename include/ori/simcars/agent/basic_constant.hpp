#pragma once

#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/constant_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class BasicConstant : public AConstant<T>
{
    const std::string entity_name;
    const std::string parameter_name;

    const T value;

public:
    BasicConstant(const std::string& entity_name, const std::string& parameter_name, T value) :
        entity_name(entity_name), parameter_name(parameter_name), value(value) {}

    std::shared_ptr<IValuelessConstant> valueless_shallow_copy() const
    {
        return shallow_copy();
    }

    std::shared_ptr<IConstant<T>> shallow_copy() const
    {
        return std::shared_ptr<BasicConstant<T>>(new BasicConstant<T>(entity_name, parameter_name, value));
    }

    std::string get_entity_name() const override
    {
        return entity_name;
    }

    std::string get_parameter_name() const override
    {
        return parameter_name;
    }

    T get_value() const override
    {
        return value;
    }
};

}
}
}

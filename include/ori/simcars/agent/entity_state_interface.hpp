#pragma once

#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IEntityState
{
public:
    virtual ~IEntityState() = default;

    virtual structures::IArray<IValuelessConstant const*>* get_parameter_values() const = 0;
    virtual IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const = 0;

    virtual void set_parameter_values(structures::IArray<IValuelessConstant const*> const *parameter_values) = 0;
    virtual void set_parameter_value(IValuelessConstant const *parameter_value) = 0;
};

}
}
}

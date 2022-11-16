#pragma once

#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IState
{
public:
    virtual ~IState() = default;

    virtual structures::IArray<IValuelessConstant const*>* get_parameter_values() const = 0;
    virtual IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const = 0;
};

}
}
}

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

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_parameter_values() const = 0;
    virtual std::shared_ptr<const IValuelessConstant> get_parameter_value(const std::string& parameter_name) const = 0;
};

}
}
}

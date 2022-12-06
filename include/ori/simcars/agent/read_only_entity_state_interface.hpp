#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IReadOnlyEntityState
{
public:
    virtual ~IReadOnlyEntityState() = default;

    virtual std::string get_name() const = 0;
    virtual temporal::Time get_time() const = 0;

    virtual bool is_populated() const = 0;

    virtual structures::IArray<IValuelessConstant const*>* get_parameter_values() const = 0;
    virtual IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const = 0;
};

}
}
}

#pragma once

#include <ori/simcars/structures/array_interface.hpp>

#include <string>
#include <memory>

namespace ori
{
namespace simcars
{
namespace agent
{

class IValuelessConstant
{
public:
    virtual ~IValuelessConstant() = default;

    virtual std::shared_ptr<IValuelessConstant> valueless_shallow_copy() const = 0;

    virtual std::string get_full_name() const = 0;
    virtual std::string get_entity_name() const = 0;
    virtual std::string get_parameter_name() const = 0;

    virtual std::string get_value_as_string() const = 0;
};

}
}
}

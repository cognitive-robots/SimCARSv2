#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>
#include <ori/simcars/agent/valueless_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IEntity
{
public:
    virtual ~IEntity() = default;

    virtual std::shared_ptr<IEntity> deep_copy() const = 0;

    virtual std::string get_name() const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constant_parameters() const = 0;
    virtual std::shared_ptr<const IValuelessConstant> get_constant_parameter(const std::string& constant_name) const = 0;
    virtual bool add_constant_parameter(const std::string& constant_name, std::shared_ptr<const IValuelessConstant> valueless_constant) = 0;
    virtual bool remove_constant_parameter(const std::string& constant_name) = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variable_parameters() const = 0;
    virtual std::shared_ptr<const IValuelessVariable> get_variable_parameter(const std::string& variable_name) const = 0;
    virtual bool add_variable_parameter(const std::string& variable_name, std::shared_ptr<const IValuelessVariable> valueless_variable) = 0;
    virtual bool remove_variable_parameter(const std::string& variable_name) = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const = 0;
    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events(temporal::Time time) const = 0;
};

}
}
}

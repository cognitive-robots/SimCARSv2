#pragma once

#include <ori/simcars/agent/read_only_entity_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IEntityState : public virtual IReadOnlyEntityState
{
public:
    virtual structures::IArray<IValuelessConstant*>* get_mutable_parameter_values() = 0;
    virtual IValuelessConstant* get_mutable_parameter_value(std::string const &parameter_name) = 0;
};

}
}
}

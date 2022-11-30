#pragma once

#include <ori/simcars/agent/valueless_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class AValuelessVariable : public virtual IValuelessVariable
{
public:
    std::string get_full_name() const override;
    std::string get_parameter_name() const override;
    std::string get_type_name() const override;
};

}
}
}

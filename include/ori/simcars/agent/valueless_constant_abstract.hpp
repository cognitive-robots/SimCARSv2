#pragma once

#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class AValuelessConstant : public virtual IValuelessConstant
{
public:
    std::string get_full_name() const override;
};

}
}
}

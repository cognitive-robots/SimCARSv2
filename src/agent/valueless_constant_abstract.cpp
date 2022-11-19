
#include <ori/simcars/agent/valueless_constant_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::string AValuelessConstant::get_full_name() const
{
    return this->get_entity_name() + "." + this->get_parameter_name();
}

}
}
}

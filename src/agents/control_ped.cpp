
#include <ori/simcars/agents/control_ped.hpp>

#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

ControlPed::ControlPed() :
    ped(nullptr),

    action(),
    action_buff(&action, nullptr, true),

    node_goal(&action_buff),
    node_val_goal(&node_goal),
    node_time_goal(&node_goal),

    ped_force()
{
}

Ped* ControlPed::get_ped()
{
    return ped;
}

void ControlPed::set_ped(Ped *ped)
{
    if (this->ped != nullptr)
    {
        this->ped->other_force.set_parent(nullptr);
        this->ped->other_force_buff.set_axiomatic(true);
    }
    if (ped != nullptr)
    {
        this->ped = ped;

        ped->other_force.set_parent(&ped_force);
        ped->other_force_buff.set_axiomatic(false);

        init_links();
    }
}

}
}
}

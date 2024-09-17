
#include <ori/simcars/agents/plan_ped.hpp>

#include <ori/simcars/agents/control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PlanPed::PlanPed() :
    control_ped(nullptr),

    action()
{
}

ControlPed* PlanPed::get_control_ped()
{
    return control_ped;
}

void PlanPed::set_control_ped(ControlPed *control_ped)
{
    if (this->control_ped != nullptr)
    {
        control_ped->action.set_parent(nullptr);
        control_ped->action_buff.set_axiomatic(true);
    }
    if (control_ped != nullptr)
    {
        this->control_ped = control_ped;

        control_ped->action.set_parent(&action);
        control_ped->action_buff.set_axiomatic(false);

        init_links();
    }
}

}
}
}

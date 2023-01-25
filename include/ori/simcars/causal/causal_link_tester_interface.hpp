#pragma once

#include <ori/simcars/agent/event_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

template <typename T1, typename T2>
class ICausalLinkTester
{
public:
    virtual ~ICausalLinkTester() = default;

    virtual void test_causal_link(agent::IScene const *scene, agent::IEvent<T1> const *cause,
                                  agent::IEvent<T2> const *effect, bool &reward_found,
                                  bool &agency_found, bool &hybrid_found) const = 0;
};

}
}
}

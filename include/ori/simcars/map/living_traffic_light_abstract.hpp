#pragma once

#include <ori/simcars/map/traffic_light_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T_id>
class ALivingTrafficLight : public ATrafficLight<T_id>
{
public:
    ALivingTrafficLight(T_id const &id, IMap<T_id> const *map) : ATrafficLight<T_id>(id, map) {}

    ITrafficLight<T_id> const* get_true_self() const noexcept override
    {
        return this->shared_from_this();
    }
};

}
}
}

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
    ALivingTrafficLight(const T_id& id, std::shared_ptr<const IMap<T_id>> map) : ATrafficLight<T_id>(id, map) {}

    std::shared_ptr<const ITrafficLight<T_id>> get_true_self() const noexcept override
    {
        return std::shared_ptr<const ITrafficLight<T_id>>(this);
    }
};

}
}
}

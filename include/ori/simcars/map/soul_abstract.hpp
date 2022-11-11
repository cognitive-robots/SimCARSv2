#pragma once

#include <ori/simcars/map/soul_interface.hpp>

namespace ori
{
namespace simcars
{
namespace map
{

template<typename T>
class ASoul : public virtual ISoul<T>
{
public:
    T const* get_self() const override
    {
        return this->get_true_self();
    }
    void banish() const override {}
};

}
}
}

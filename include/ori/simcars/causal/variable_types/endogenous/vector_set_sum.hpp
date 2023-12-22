#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/set_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorSetSumVariable : public ASetEndogenousVariable<geometry::Vec, geometry::Vec>
{
public:
    using ASetEndogenousVariable<geometry::Vec, geometry::Vec>::ASetEndogenousVariable;

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}

#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorTimeConditionalVariable :
        public ABinaryEndogenousVariable<geometry::Vec, geometry::Vec, geometry::Vec>
{
    temporal::Time const time;

public:
    VectorTimeConditionalVariable(IEndogenousVariable<geometry::Vec> *endogenous_parent,
                                  IVariable<geometry::Vec> *other_parent, temporal::Time time);

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}

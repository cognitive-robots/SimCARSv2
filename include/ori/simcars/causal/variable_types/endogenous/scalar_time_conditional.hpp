#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarTimeConditionalVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
    temporal::Time const time;

public:
    ScalarTimeConditionalVariable(IEndogenousVariable<FP_DATA_TYPE> *endogenous_parent,
                                  IVariable<FP_DATA_TYPE> *other_parent, temporal::Time time);

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}

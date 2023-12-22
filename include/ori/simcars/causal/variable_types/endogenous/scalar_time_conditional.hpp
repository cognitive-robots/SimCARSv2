#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarTimeConditionalVariable :
        public ATernaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE, temporal::Time>
{
public:
    using ATernaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE,
    temporal::Time>::ATernaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}

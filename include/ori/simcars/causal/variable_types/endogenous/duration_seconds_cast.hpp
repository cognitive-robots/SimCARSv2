#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class DurationSecondsCastVariable : public AUnaryEndogenousVariable<FP_DATA_TYPE, temporal::Duration>
{
public:
    using AUnaryEndogenousVariable<FP_DATA_TYPE, temporal::Duration>::AUnaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}

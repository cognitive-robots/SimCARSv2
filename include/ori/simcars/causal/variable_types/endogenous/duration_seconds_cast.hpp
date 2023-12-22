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

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}

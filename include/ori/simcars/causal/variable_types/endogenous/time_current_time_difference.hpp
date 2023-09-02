#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class TimeCurrentTimeDifferenceVariable :
        public AUnaryEndogenousVariable<temporal::Duration, temporal::Time>
{
public:
    using AUnaryEndogenousVariable<temporal::Duration, temporal::Time>::AUnaryEndogenousVariable;

    temporal::Duration get_value() const override;
};

}
}
}

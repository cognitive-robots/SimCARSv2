#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/set_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarSetSumVariable : public ASetEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ASetEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>::ASetEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}

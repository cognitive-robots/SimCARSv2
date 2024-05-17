#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/set_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarSetMinVariable : public ASetEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ASetEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>::ASetEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}

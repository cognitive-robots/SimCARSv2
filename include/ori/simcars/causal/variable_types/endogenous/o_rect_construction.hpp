#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/quaternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ORectConstructionVariable :
        public AQuaternaryEndogenousVariable<geometry::ORect, geometry::Vec, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using AQuaternaryEndogenousVariable<geometry::ORect, geometry::Vec, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>::AQuaternaryEndogenousVariable;

    geometry::ORect get_value() const override;
};

}
}
}

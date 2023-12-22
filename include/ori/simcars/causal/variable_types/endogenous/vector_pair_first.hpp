#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorPairFirstVariable : public AUnaryEndogenousVariable<geometry::Vec, geometry::VecPair>
{
public:
    using AUnaryEndogenousVariable<geometry::Vec, geometry::VecPair>::AUnaryEndogenousVariable;

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;
};

}
}
}

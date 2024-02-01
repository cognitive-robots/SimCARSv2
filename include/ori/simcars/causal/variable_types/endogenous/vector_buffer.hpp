#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorBufferVariable : public AUnaryEndogenousVariable<geometry::Vec, geometry::Vec>
{
    temporal::TemporalDictionary<geometry::Vec>* temporal_dictionary;

    bool axiomatic;

public:
    VectorBufferVariable(
            IVariable<geometry::Vec> *parent,
            temporal::TemporalDictionary<geometry::Vec> *temporal_dictionary = nullptr,
            bool axiomatic = false);

    ~VectorBufferVariable();

    bool get_value(geometry::Vec &val) const override;

    bool set_value(geometry::Vec const &val) override;

    temporal::Time get_min_time() const;
    temporal::Time get_max_time() const;

    void set_axiomatic(bool axiomatic);
};

}
}
}

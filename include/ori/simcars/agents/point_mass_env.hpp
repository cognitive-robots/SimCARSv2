#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_exponent.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_set_sum.hpp>
#include <ori/simcars/agents/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PointMassEnv
{
    class PointMassEntity
    {
        class PointMassLink
        {
            uint64_t id;
            PointMass *point_mass;
            uint64_t other_id;
            PointMass *other_point_mass;

        protected:
            simcars::causal::VectorNegationVariable neg_other_pos;
            simcars::causal::VectorSumVariable pos_diff;

            simcars::causal::ScalarFixedVariable dist_scale_factor;
            simcars::causal::VectorNormVariable dist;
            simcars::causal::ScalarProductVariable scaled_dist;
            simcars::causal::ScalarNegationVariable neg_scaled_dist;
            simcars::causal::ScalarExponentVariable force_mag;

            simcars::causal::VectorNormalisationVariable force_dir;
            simcars::causal::VectorScalarProductVariable actual_avoidance_force;

        public:
            PointMassLink(uint64_t id, PointMass *point_mass, uint64_t other_id,
                 PointMass *other_point_mass);

            simcars::causal::IEndogenousVariable<geometry::Vec>* get_avoidance_force();
        };

        uint64_t id;
        PointMass *point_mass;
        structures::stl::STLDictionary<uint64_t, PointMassLink*> id_link_dict;

    protected:
        simcars::causal::VectorSetSumVariable env_force;

    public:
        PointMassEntity(uint64_t id, PointMass *point_mass);

        virtual ~PointMassEntity();

        simcars::causal::IEndogenousVariable<geometry::Vec>* get_env_force();

        bool add_link(PointMass *other_point_mass);
        bool remove_link(PointMass *other_point_mass);
    };

    structures::stl::STLDictionary<uint64_t, PointMass*> id_point_mass_dict;
    structures::stl::STLDictionary<uint64_t, PointMassEntity*> id_entity_dict;

public:
    virtual ~PointMassEnv();

    structures::IArray<PointMass*> const* get_point_masses() const;

    bool add_point_mass(PointMass *point_mass);
    bool remove_point_mass(PointMass *point_mass);
};

}
}
}

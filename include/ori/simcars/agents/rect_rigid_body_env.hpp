#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_quotient.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_conditional.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_set_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_dot_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_cross_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_conditional.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_first.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_pair_second.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_set_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/o_rect_collision.hpp>
#include <ori/simcars/causal/variable_types/endogenous/o_rect_contact.hpp>
#include <ori/simcars/agents/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class RectRigidBodyEnv
{
    class Entity
    {
        class Link
        {
        protected:
            causal::ScalarSumVariable mass_sum;
            causal::ScalarReciprocalVariable mass_sum_recip;

            causal::ORectCollisionVariable coll;

            causal::ORectContactVariable coll_contact;
            causal::VectorPairFirstVariable coll_contact_pos;
            causal::VectorPairSecondVariable coll_contact_dir;

            causal::VectorDotProductVariable ali_lin_vel_mag;
            causal::VectorDotProductVariable other_ali_lin_vel_mag;

            causal::ScalarProductVariable ali_lin_mom_mag;
            causal::ScalarProductVariable other_ali_lin_mom_mag;
            causal::ScalarSumVariable ali_lin_mom_mag_sum;
            causal::ScalarProductVariable coll_lin_vel_mag;

            causal::ScalarNegationVariable neg_ali_lin_vel_mag;

            causal::ScalarSumVariable coll_lin_vel_mag_diff;
            causal::ScalarTimeStepSizeQuotientVariable coll_lin_acc_mag;
            causal::ScalarProductVariable coll_force_mag;
            causal::VectorScalarProductVariable coll_force;

            causal::VectorFixedVariable no_coll_force;
            causal::VectorProxyVariable no_coll_force_proxy;

            causal::VectorConditionalVariable actual_coll_force;

            causal::VectorNegationVariable neg_pos;
            causal::VectorSumVariable coll_contact_rel_pos;

            causal::VectorCrossProductVariable coll_torque;

            causal::ScalarFixedVariable no_coll_torque;
            causal::ScalarProxyVariable no_coll_torque_proxy;

            causal::ScalarConditionalVariable actual_coll_torque;

        public:
            Link(RectRigidBody const *rigid_body, RectRigidBody const *other_rigid_body);

            causal::IEndogenousVariable<geometry::Vec> const* get_coll_force() const;
            causal::IEndogenousVariable<FP_DATA_TYPE> const* get_coll_torque() const;
        };

        RectRigidBody const *rigid_body;
        structures::stl::STLDictionary<RectRigidBody const*, Link*> other_rigid_body_link_dict;

    protected:
        causal::ScalarFixedVariable half_scale_factor;
        causal::ScalarProxyVariable half_scale_factor_proxy;

        causal::ScalarFixedVariable air_mass_density;
        causal::ScalarProductVariable drag_scaled_air_mass_density;

        causal::VectorNormalisationVariable lin_vel_dir;
        causal::VectorNegationVariable drag_force_dir;

        causal::VectorDotProductVariable lin_spd_squared;
        causal::ScalarProductVariable dynamic_pressure;
        causal::ScalarProductVariable drag_force_mag;

        causal::VectorScalarProductVariable drag_force;
        causal::ScalarFixedVariable drag_torque;
        causal::ScalarProxyVariable drag_torque_proxy;

        causal::VectorSetSumVariable env_force;
        causal::ScalarSetSumVariable env_torque;

    public:
        Entity(RectRigidBody const *rigid_body);

        virtual ~Entity();

        causal::IEndogenousVariable<geometry::Vec> const* get_env_force() const;
        causal::IEndogenousVariable<FP_DATA_TYPE> const* get_env_torque() const;

        bool add_link(RectRigidBody const *other_rigid_body);
        bool remove_link(RectRigidBody const *other_rigid_body);
    };

    structures::stl::STLDictionary<RectRigidBody const*, Entity*> rigid_body_entity_dict;

public:
    virtual ~RectRigidBodyEnv();

    bool add_entity(RectRigidBody *rigid_body);
    bool remove_entity(RectRigidBody *rigid_body);
};

}
}
}

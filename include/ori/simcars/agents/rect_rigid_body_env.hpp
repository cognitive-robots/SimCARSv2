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
            uint64_t id;
            RectRigidBody *rigid_body;
            uint64_t other_id;
            RectRigidBody *other_rigid_body;

        protected:
            simcars::causal::ScalarSumVariable mass_sum;
            simcars::causal::ScalarReciprocalVariable mass_sum_recip;

            simcars::causal::ORectCollisionVariable coll;

            simcars::causal::ORectContactVariable coll_contact;
            simcars::causal::VectorPairFirstVariable coll_contact_pos;
            simcars::causal::VectorPairSecondVariable coll_contact_dir;

            simcars::causal::VectorDotProductVariable ali_lin_vel_mag;
            simcars::causal::VectorDotProductVariable other_ali_lin_vel_mag;

            simcars::causal::ScalarProductVariable ali_lin_mom_mag;
            simcars::causal::ScalarProductVariable other_ali_lin_mom_mag;
            simcars::causal::ScalarSumVariable ali_lin_mom_mag_sum;
            simcars::causal::ScalarProductVariable coll_lin_vel_mag;

            simcars::causal::ScalarNegationVariable neg_ali_lin_vel_mag;

            simcars::causal::ScalarSumVariable coll_lin_vel_mag_diff;
            simcars::causal::ScalarTimeStepSizeQuotientVariable coll_lin_acc_mag;
            simcars::causal::ScalarProductVariable coll_force_mag;
            simcars::causal::VectorScalarProductVariable coll_force;

            simcars::causal::VectorFixedVariable no_coll_force;
            simcars::causal::VectorProxyVariable no_coll_force_proxy;

            simcars::causal::VectorConditionalVariable actual_coll_force;

            simcars::causal::VectorNegationVariable neg_pos;
            simcars::causal::VectorSumVariable coll_contact_rel_pos;

            simcars::causal::VectorCrossProductVariable coll_torque;

            simcars::causal::ScalarFixedVariable no_coll_torque;
            simcars::causal::ScalarProxyVariable no_coll_torque_proxy;

            simcars::causal::ScalarConditionalVariable actual_coll_torque;

        public:
            Link(uint64_t id, RectRigidBody *rigid_body, uint64_t other_id,
                 RectRigidBody *other_rigid_body);

            simcars::causal::IEndogenousVariable<geometry::Vec>* get_coll_force();
            simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_coll_torque();
        };

        uint64_t id;
        RectRigidBody *rigid_body;
        structures::stl::STLDictionary<uint64_t, Link*> id_link_dict;

    protected:
        simcars::causal::ScalarFixedVariable half_scale_factor;
        simcars::causal::ScalarProxyVariable half_scale_factor_proxy;

        simcars::causal::ScalarFixedVariable air_mass_density;
        simcars::causal::ScalarProductVariable drag_scaled_air_mass_density;

        simcars::causal::VectorNormalisationVariable lin_vel_dir;
        simcars::causal::VectorNegationVariable drag_force_dir;

        simcars::causal::VectorDotProductVariable lin_spd_squared;
        simcars::causal::ScalarProductVariable dynamic_pressure;
        simcars::causal::ScalarProductVariable drag_force_mag;

        simcars::causal::VectorScalarProductVariable drag_force;
        simcars::causal::ScalarFixedVariable drag_torque;
        simcars::causal::ScalarProxyVariable drag_torque_proxy;

        simcars::causal::VectorSetSumVariable env_force;
        simcars::causal::ScalarSetSumVariable env_torque;

    public:
        Entity(uint64_t id, RectRigidBody *rigid_body);

        virtual ~Entity();

        simcars::causal::IEndogenousVariable<geometry::Vec>* get_env_force();
        simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_env_torque();

        bool add_link(RectRigidBody *other_rigid_body);
        bool remove_link(RectRigidBody *other_rigid_body);
    };

    structures::stl::STLDictionary<uint64_t, RectRigidBody*> id_rigid_body_dict;
    structures::stl::STLDictionary<uint64_t, Entity*> id_entity_dict;

public:
    virtual ~RectRigidBodyEnv();

    structures::IArray<RectRigidBody*> const* get_rigid_bodies() const;

    bool add_rigid_body(RectRigidBody *rigid_body);
    bool remove_rigid_body(RectRigidBody *rigid_body);
};

}
}
}

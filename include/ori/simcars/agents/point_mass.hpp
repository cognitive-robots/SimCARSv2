#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_binary_mean.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_binary_mean.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/o_rect_construction.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/point_mass_env.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PointMass
{
protected:
    simcars::causal::IdFixedVariable id;
    simcars::causal::IdProxyVariable id_proxy;

    simcars::causal::ScalarFixedVariable mass;
    simcars::causal::ScalarProxyVariable mass_proxy;
    simcars::causal::ScalarReciprocalVariable mass_recip;

    // TODO: Move neighbour distance related elements to the pedestrian class
    simcars::causal::ScalarSocketVariable min_neighbour_dist;
    simcars::causal::ScalarBufferVariable min_neighbour_dist_buff;

    simcars::causal::VectorSocketVariable env_force;
    simcars::causal::VectorBufferVariable env_force_buff;
    simcars::causal::VectorSocketVariable other_force;
    simcars::causal::VectorBufferVariable other_force_buff;
    simcars::causal::VectorSumVariable total_force;

    simcars::causal::VectorScalarProductVariable lin_acc;
    simcars::causal::VectorBufferVariable lin_acc_buff;
    simcars::causal::VectorPreviousTimeStepVariable prev_lin_acc;

    simcars::causal::VectorTimeStepSizeProductVariable lin_vel_diff;
    simcars::causal::VectorPreviousTimeStepVariable prev_lin_vel;
    simcars::causal::VectorSumVariable lin_vel;
    simcars::causal::VectorBufferVariable lin_vel_buff;

    simcars::causal::VectorTimeStepSizeProductVariable pos_diff;
    simcars::causal::VectorPreviousTimeStepVariable prev_pos;
    simcars::causal::VectorSumVariable pos;
    simcars::causal::VectorBufferVariable pos_buff;

public:
    PointMass(uint64_t id_value, FP_DATA_TYPE mass_value);
    PointMass(PointMass const &point_mass);

    virtual ~PointMass() = default;

    simcars::causal::IEndogenousVariable<uint64_t>* get_id_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_mass_variable();

    virtual temporal::Time get_min_time() const;
    virtual temporal::Time get_max_time() const;

    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_min_neighbour_dist_variable();
    virtual simcars::causal::IEndogenousVariable<geometry::Vec>* get_env_force_variable();
    virtual simcars::causal::IEndogenousVariable<geometry::Vec>* get_other_force_variable();
    virtual simcars::causal::IEndogenousVariable<geometry::Vec>* get_lin_acc_variable();
    virtual simcars::causal::IEndogenousVariable<geometry::Vec>* get_lin_vel_variable();
    virtual simcars::causal::IEndogenousVariable<geometry::Vec>* get_pos_variable();

    friend bool PointMassEnv::add_point_mass(PointMass *point_mass);
    friend bool PointMassEnv::remove_point_mass(PointMass *point_mass);
};

}
}
}

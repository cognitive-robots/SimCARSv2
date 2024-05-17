
#include <ori/simcars/causal/variable_types/endogenous/o_rect_dist_headway.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ORectDistHeadwayVariable::ORectDistHeadwayVariable(IEndogenousVariable<geometry::ORect> *endogenous_parent,
                                                   IVariable<geometry::ORect> *other_parent) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent), trig_buff(geometry::TrigBuff::get_instance()) {}

bool ORectDistHeadwayVariable::get_value(FP_DATA_TYPE &val) const
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        geometry::Vec pos_diff = rect_2.get_origin() - rect_1.get_origin();

        FP_DATA_TYPE sin_o_1 = trig_buff->get_sin(rect_1.get_orientation());
        FP_DATA_TYPE cos_o_1 = trig_buff->get_cos(rect_1.get_orientation());
        geometry::Vec dir_1(cos_o_1, sin_o_1);

        FP_DATA_TYPE pos_diff_proj = dir_1.dot(pos_diff);

        if (pos_diff_proj <= 0.0)
        {
            val = 100.0;
            return true;
        }

        // WARNING: This assumes the vehicles are similarly oriented, which may not always be the
        // case, it is fine however for highway scenarios
        FP_DATA_TYPE collision_span = rect_1.get_half_height() + rect_2.get_half_height();

        if (pos_diff.squaredNorm() < (std::pow(pos_diff_proj, 2) + std::pow(collision_span, 2)))
        {
            val = std::min(pos_diff_proj, 100.0);
        }
        else
        {
            val = 100.0;
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool ORectDistHeadwayVariable::set_value(FP_DATA_TYPE const &val)
{
    geometry::ORect rect_1, rect_2;
    if (get_endogenous_parent()->get_value(rect_1) && get_other_parent()->get_value(rect_2))
    {
        geometry::Vec pos_diff = rect_2.get_origin() - rect_1.get_origin();

        FP_DATA_TYPE sin_o_1 = trig_buff->get_sin(rect_1.get_orientation());
        FP_DATA_TYPE cos_o_1 = trig_buff->get_cos(rect_1.get_orientation());
        geometry::Vec dir_1(cos_o_1, sin_o_1);

        FP_DATA_TYPE pos_diff_proj = dir_1.dot(pos_diff);

        if (pos_diff_proj <= 0.0)
        {
            return val == 100.0;
        }

        // WARNING: This assumes the vehicles are similarly oriented, which may not always be the
        // case, it is fine however for highway scenarios
        FP_DATA_TYPE collision_span = rect_1.get_half_width() + rect_2.get_half_width();

        if (pos_diff.norm() < (std::pow(pos_diff_proj, 2) + std::pow(collision_span, 2)))
        {
            return val == std::min(pos_diff_proj, 100.0);
        }
        else
        {
            return val == 100.0;
        }
    }
    else
    {
        return true;
    }
}

}
}
}

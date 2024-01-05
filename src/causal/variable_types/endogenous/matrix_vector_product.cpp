
#include <ori/simcars/causal/variable_types/endogenous/matrix_vector_product.hpp>

#include <Eigen/LU>

namespace ori
{
namespace simcars
{
namespace causal
{

bool MatrixVectorProductVariable::get_value(geometry::Vec &val) const
{
    geometry::RotMat rot_mat;
    if (get_endogenous_parent()->get_value(rot_mat) && get_other_parent()->get_value(val))
    {
        val = rot_mat * val;
        return true;
    }
    else
    {
        return false;
    }
}

bool MatrixVectorProductVariable::set_value(geometry::Vec const &val)
{
    geometry::RotMat rot_mat;
    geometry::Vec vec;
    if (get_endogenous_parent()->get_value(rot_mat))
    {
        if (get_other_parent()->get_value(vec))
        {
            return (rot_mat * vec) == val;
        }
        else
        {
            return get_other_parent()->set_value(rot_mat.inverse() * val);
        }
    }
    else
    {
        if (get_other_parent()->get_value(vec))
        {
            geometry::RotMat rot_mat;
            FP_DATA_TYPE sin_a = std::abs(vec.x() * val.y() - vec.y() * val.x());
            FP_DATA_TYPE cos_a = vec.dot(val);
            rot_mat << cos_a, -sin_a, sin_a, cos_a;
            return get_endogenous_parent()->set_value(rot_mat);
        }
        else
        {
            return true;
        }
    }
}

}
}
}

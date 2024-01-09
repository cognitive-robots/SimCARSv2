
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace highd
{

HighDFWDCarScene::HighDFWDCarScene(std::string const &tracks_meta_path_str, std::string const &tracks_path_str)
    : HighDFWDCarScene(rapidcsv::Document(tracks_meta_path_str), rapidcsv::Document(tracks_path_str)) {}

HighDFWDCarScene::HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc, rapidcsv::Document const &tracks_doc)
{
    min_time = temporal::Time::max();
    max_time = temporal::Time::min();

    size_t i, j = 0, k;
    for (i = 0; i < tracks_meta_doc.GetRowCount(); ++i)
    {
        uint32_t const id = tracks_meta_doc.GetCell<uint32_t>("id", i);

        // Note: Length/width in local frame vs width/height in global frame
        FP_DATA_TYPE length = tracks_meta_doc.GetCell<FP_DATA_TYPE>("width", i);
        FP_DATA_TYPE width = tracks_meta_doc.GetCell<FP_DATA_TYPE>("height", i);

        // Several big approximations here based off Toyota Ascent Sport (Hybrid), 1.8L
        FP_DATA_TYPE approx_height = 0.806 * width;
        FP_DATA_TYPE approx_mass = 116 * length * width * approx_height;
        FP_DATA_TYPE approx_axel_dist = 0.292 * length;
        FP_DATA_TYPE approx_wheel_radius = 0.19;

        uint32_t initial_frame = tracks_meta_doc.GetCell<uint32_t>("initialFrame", i);
        uint32_t final_frame = tracks_meta_doc.GetCell<uint32_t>("finalFrame", i);

        min_time = std::min(temporal::Time(temporal::Duration(((initial_frame - 1) *
                                                               get_time_step_size()))), min_time);
        max_time = std::max(temporal::Time(temporal::Duration(((final_frame - 1) *
                                                               get_time_step_size()))), max_time);

        for (; j < tracks_doc.GetRowCount(); ++j)
        {
            if (tracks_doc.GetCell<uint32_t>("id", j) == id)
            {
                break;
            }
        }
        if (j == tracks_doc.GetRowCount())
        {
            throw std::runtime_error("Could not find agent id specified in tracks meta file in tracks file");
        }

        FWDCar *fwd_car = new FWDCar(approx_mass, length, width, approx_height, approx_wheel_radius,
                                     approx_axel_dist);

        simcars::causal::IEndogenousVariable<geometry::Vec> *position_variable =
                fwd_car->get_pos_variable();

        simcars::causal::IEndogenousVariable<FP_DATA_TYPE> *rotation_variable =
                fwd_car->get_rot_variable();

        simcars::causal::VariableContext::set_time_step_size(get_time_step_size());

        size_t const tracks_start_row = j;
        size_t const tracks_end_row = tracks_start_row + (final_frame - initial_frame);
        j = tracks_end_row + 1;
        for (k = tracks_start_row; k <= tracks_end_row; ++k)
        {
            uint32_t current_frame = tracks_doc.GetCell<uint32_t>("frame", k);
            temporal::Time current_time = temporal::Time(temporal::Duration(((current_frame - 1) *
                                                                             get_time_step_size())));

            simcars::causal::VariableContext::set_current_time(current_time);

            FP_DATA_TYPE const position_x = tracks_doc.GetCell<FP_DATA_TYPE>("x", k) + length / 2.0f;
            FP_DATA_TYPE const position_y = tracks_doc.GetCell<FP_DATA_TYPE>("y", k) + width / 2.0f;
            geometry::Vec const position(position_x, position_y);

            position_variable->set_value(position);
            rotation_variable->set_value(0);
        }

        id_car_dict.update(id, fwd_car);
    }
}

HighDFWDCarScene::~HighDFWDCarScene()
{
    structures::IArray<FWDCar*> const *cars = id_car_dict.get_values();
    for (size_t i = 0; i < cars->count(); ++i)
    {
        delete (*cars)[i];
    }
}

temporal::Duration HighDFWDCarScene::get_time_step_size() const
{
    return temporal::Duration(40);
}

temporal::Time HighDFWDCarScene::get_min_time() const
{
    return min_time;
}

temporal::Time HighDFWDCarScene::get_max_time() const
{
    return max_time;
}

structures::IArray<FWDCar*> const* HighDFWDCarScene::get_fwd_cars() const
{
    return id_car_dict.get_values();
}

}
}
}
}

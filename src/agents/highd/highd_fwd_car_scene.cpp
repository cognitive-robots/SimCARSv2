
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

HighDFWDCarScene::HighDFWDCarScene(std::string const &tracks_meta_path_str,
                                   std::string const &tracks_path_str, map::IMap const *map)
    : HighDFWDCarScene(rapidcsv::Document(tracks_meta_path_str),
                       rapidcsv::Document(tracks_path_str), map) {}

HighDFWDCarScene::HighDFWDCarScene(std::string const &tracks_meta_path_str,
                                   std::string const &tracks_path_str, size_t start_frame,
                                   size_t end_frame, map::IMap const *map)
    : HighDFWDCarScene(rapidcsv::Document(tracks_meta_path_str),
                       rapidcsv::Document(tracks_path_str), start_frame, end_frame, map) {}

HighDFWDCarScene::HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc,
                                   rapidcsv::Document const &tracks_doc, map::IMap const *map)
    : HighDFWDCarScene(tracks_meta_doc, tracks_doc, 0, -1, map) {}

HighDFWDCarScene::HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc,
                                   rapidcsv::Document const &tracks_doc, size_t start_frame,
                                   size_t end_frame, map::IMap const *map)
{
    size_t min_frame = std::numeric_limits<size_t>::max();
    size_t max_frame = std::numeric_limits<size_t>::min();

    size_t i, j = 0, k;
    for (i = 0; i < tracks_meta_doc.GetRowCount(); ++i)
    {
        uint32_t const id = tracks_meta_doc.GetCell<uint32_t>("id", i);

        // Note: Length/width in local frame vs width/height in global frame
        FP_DATA_TYPE length = tracks_meta_doc.GetCell<FP_DATA_TYPE>("width", i);
        FP_DATA_TYPE width = tracks_meta_doc.GetCell<FP_DATA_TYPE>("height", i);

        size_t initial_frame = tracks_meta_doc.GetCell<size_t>("initialFrame", i);
        size_t final_frame = tracks_meta_doc.GetCell<size_t>("finalFrame", i);

        if ((start_frame < end_frame) && (initial_frame > end_frame || final_frame < start_frame))
        {
            continue;
        }

        min_frame = std::min(initial_frame, min_frame);
        max_frame = std::max(final_frame, max_frame);

        // Several big approximations here based off Toyota Ascent Sport (Hybrid), 1.8L
        FP_DATA_TYPE approx_height = 0.806 * width;
        FP_DATA_TYPE approx_mass = 116 * length * width * approx_height;
        FP_DATA_TYPE approx_axle_dist = 0.292 * length;
        FP_DATA_TYPE approx_wheel_radius = 0.19;

        FWDCar *fwd_car = new FWDCar(id, approx_mass, length, width, approx_height,
                                     approx_wheel_radius, approx_axle_dist);
        id_car_dict.update(id, fwd_car);
        scene_env.add_rigid_body(fwd_car);

        /*
        if (map != nullptr)
        {
            // TODO: Replace hardcoded values for thresholds
            FullControlFWDCar *control_fwd_car = new FullControlFWDCar(map, 163, 0, 0.616);
            control_fwd_car->set_fwd_car(fwd_car);
            id_control_dict.update(id, control_fwd_car);

            // TODO: Reuse simulator and reward calculator
            IFWDCarOutcomeSim *outcome_sim = new DefaultFWDCarOutcomeSim(control_fwd_car,
                                                                         &scene_env);
            FWDCarSimParameters outcome_sim_params = { 5 };
            IFWDCarRewardCalc *reward_calc = new DefaultFWDCarRewardCalc();
            FWDCarRewardParameters reward_calc_params = { 35.8, 2100 };
            PlanFWDCar *plan_fwd_car = new GreedyPlanFWDCar(map, outcome_sim, outcome_sim_params,
                                                            reward_calc, reward_calc_params, 0,
                                                            45, 5, 10, 2);
            plan_fwd_car->set_control_fwd_car(control_fwd_car);
            id_plan_dict.update(id, plan_fwd_car);
        }
        */
    }

    if (start_frame == 0 && end_frame == -1)
    {
        start_frame = min_frame;
        end_frame = max_frame;
    }
    else
    {
        min_frame = std::max(start_frame, min_frame);
        max_frame = std::min(end_frame, max_frame);
    }

    min_time = temporal::Time(temporal::Duration(((min_frame - 1) * get_time_step_size())));
    max_time = temporal::Time(temporal::Duration(((max_frame - 1) * get_time_step_size())));

    structures::stl::STLStackArray<uint32_t> id_array;
    structures::stl::STLDictionary<uint32_t, size_t> id_initial_frame_dict;
    structures::stl::STLDictionary<uint32_t, size_t> id_final_frame_dict;
    structures::stl::STLDictionary<uint32_t, structures::IArray<geometry::Vec>*> id_pos_dict;
    structures::stl::STLDictionary<uint32_t, structures::IArray<FP_DATA_TYPE>*> id_rot_dict;

    for (i = 0; i < tracks_meta_doc.GetRowCount(); ++i)
    {
        uint32_t const id = tracks_meta_doc.GetCell<uint32_t>("id", i);

        FP_DATA_TYPE length = tracks_meta_doc.GetCell<FP_DATA_TYPE>("width", i);
        FP_DATA_TYPE width = tracks_meta_doc.GetCell<FP_DATA_TYPE>("height", i);

        size_t initial_frame = tracks_meta_doc.GetCell<size_t>("initialFrame", i);
        size_t final_frame = tracks_meta_doc.GetCell<size_t>("finalFrame", i);

        if (initial_frame > end_frame || final_frame < start_frame)
        {
            continue;
        }

        id_array.push_back(id);
        id_initial_frame_dict.update(id, initial_frame);
        id_final_frame_dict.update(id, final_frame);
        id_pos_dict.update(id, new structures::stl::STLStackArray<geometry::Vec>(1 + end_frame -
                                                                                 start_frame));
        id_rot_dict.update(id, new structures::stl::STLStackArray<FP_DATA_TYPE>(1 + end_frame -
                                                                                start_frame));

        for (; j < tracks_doc.GetRowCount(); ++j)
        {
            if (tracks_doc.GetCell<uint32_t>("id", j) == id)
            {
                break;
            }
        }
        if (j == tracks_doc.GetRowCount())
        {
            throw std::runtime_error("Could not find agent id specified in tracks meta file in "
                                     "tracks file");
        }

        size_t const tracks_start_row = j;
        size_t const tracks_end_row = tracks_start_row + (final_frame - initial_frame);
        j = tracks_end_row + 1;
        for (k = tracks_start_row; k <= tracks_end_row; ++k)
        {
            uint32_t current_frame = tracks_doc.GetCell<uint32_t>("frame", k);

            if (current_frame > end_frame) break;
            if (current_frame < start_frame) continue;

            FP_DATA_TYPE const position_x = tracks_doc.GetCell<FP_DATA_TYPE>("x", k) + length / 2.0f;
            FP_DATA_TYPE const position_y = tracks_doc.GetCell<FP_DATA_TYPE>("y", k) + width / 2.0f;
            geometry::Vec const position(position_x, position_y);

            (*(id_pos_dict[id]))[current_frame - start_frame] = position;

            FP_DATA_TYPE const velocity_x = tracks_doc.GetCell<FP_DATA_TYPE>("xVelocity", k);
            FP_DATA_TYPE const velocity_y = tracks_doc.GetCell<FP_DATA_TYPE>("yVelocity", k);
            FP_DATA_TYPE const rotation = std::atan2(velocity_y, velocity_x);

            (*(id_rot_dict[id]))[current_frame - start_frame] = rotation;
        }
    }

    simcars::causal::VariableContext::set_time_step_size(get_time_step_size());

    for (i = 0; i < 1 + end_frame - start_frame; ++i)
    {
        uint32_t current_frame = start_frame + i;

        temporal::Time current_time = temporal::Time(temporal::Duration(((current_frame - 1) *
                                                                         get_time_step_size())));

        simcars::causal::VariableContext::set_current_time(current_time);

        for (j = 0; j < id_array.count(); ++j)
        {
            uint32_t id = id_array[j];

            if (current_frame < id_initial_frame_dict[id] ||
                    current_frame > id_final_frame_dict[id])
            {
                continue;
            }

            FWDCar *fwd_car = id_car_dict[id];

            simcars::causal::IEndogenousVariable<geometry::Vec> *position_variable =
                    fwd_car->get_pos_variable();

            geometry::Vec position = (*(id_pos_dict[id]))[i];
            bool res = position_variable->set_value(position);

            if (!res)
            {
                throw std::runtime_error("Failed to set position variable");
            }

            simcars::causal::IEndogenousVariable<FP_DATA_TYPE> *rotation_variable =
                    fwd_car->get_rot_variable();

            FP_DATA_TYPE rotation = (*(id_rot_dict[id]))[i];
            res = rotation_variable->set_value(rotation);

            if (!res)
            {
                throw std::runtime_error("Failed to set rotation variable");
            }
        }
    }

    for (i = 0; i < id_array.count(); ++i)
    {
        uint32_t id = id_array[i];
        delete id_pos_dict[id];
        delete id_rot_dict[id];
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

FWDCar* HighDFWDCarScene::get_fwd_car(uint32_t id) const
{
    return id_car_dict[id];
}

ControlFWDCar* HighDFWDCarScene::get_control_fwd_car(uint32_t id) const
{
    return id_control_dict[id];
}

PlanFWDCar* HighDFWDCarScene::get_plan_fwd_car(uint32_t id) const
{
    return id_plan_dict[id];
}

structures::IArray<FWDCar*> const* HighDFWDCarScene::get_fwd_cars() const
{
    return id_car_dict.get_values();
}

structures::IArray<ControlFWDCar*> const* HighDFWDCarScene::get_control_fwd_cars() const
{
    return id_control_dict.get_values();
}

structures::IArray<PlanFWDCar*> const* HighDFWDCarScene::get_plan_fwd_cars() const
{
    return id_plan_dict.get_values();
}

RectRigidBodyEnv* HighDFWDCarScene::get_env()
{
    return &scene_env;
}

}
}
}
}

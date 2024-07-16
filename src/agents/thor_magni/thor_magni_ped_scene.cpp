
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace thor_magni
{

ThorMagniPedScene::ThorMagniPedScene(std::string const &scene_path_str)
    : ThorMagniPedScene(rapidcsv::Document(scene_path_str)) {}

ThorMagniPedScene::ThorMagniPedScene(rapidcsv::Document const &scene_doc)
{
    std::hash<std::string> str_hash;

    min_time = temporal::Time::max();
    max_time = temporal::Time::min();

    simcars::causal::VariableContext::set_time_step_size(get_time_step_size());

    for (size_t i = 0; i < scene_doc.GetRowCount(); ++i)
    {
        std::string const id_str = scene_doc.GetCell<std::string>("ag_id", i);
        uint32_t const id = str_hash(id_str);

        size_t const frame = scene_doc.GetCell<size_t>("frame_id", i);
        temporal::Time const time(temporal::Duration(((frame - 1) * get_time_step_size())));
        min_time = std::min(time, min_time);
        max_time = std::max(time, max_time);

        // Approximate average human mass
        FP_DATA_TYPE const approx_mass = 70;

        PointMass *point_mass;

        if (id_mass_dict.contains(id))
        {
            point_mass = id_mass_dict[id];
        }
        else
        {
            point_mass = new PointMass(id, approx_mass);
            id_mass_dict.update(id, point_mass);
        }

        simcars::causal::VariableContext::set_current_time(time);

        simcars::causal::IEndogenousVariable<geometry::Vec>* const position_variable =
                point_mass->get_pos_variable();

        std::string const position_x_str = scene_doc.GetCell<std::string>("x", i);
        std::string const position_y_str = scene_doc.GetCell<std::string>("y", i);

        if (position_x_str != "" && position_y_str != "")
        {
            FP_DATA_TYPE const position_x = std::strtod(position_x_str.c_str(), nullptr) / 1000.0;
            FP_DATA_TYPE const position_y = std::strtod(position_y_str.c_str(), nullptr) / 1000.0;
            geometry::Vec const position(position_x, position_y);

            bool res = position_variable->set_value(position);

            if (!res)
            {
                throw std::runtime_error("Failed to set position variable");
            }
        }
    }
}

ThorMagniPedScene::~ThorMagniPedScene()
{
    structures::IArray<PointMass*> const *masses = id_mass_dict.get_values();
    for (size_t i = 0; i < masses->count(); ++i)
    {
        delete (*masses)[i];
    }
}

temporal::Duration ThorMagniPedScene::get_time_step_size() const
{
    return temporal::Duration(200);
}

temporal::Time ThorMagniPedScene::get_min_time() const
{
    return min_time;
}

temporal::Time ThorMagniPedScene::get_max_time() const
{
    return max_time;
}

PointMass* ThorMagniPedScene::get_point_mass(uint32_t id) const
{
    return id_mass_dict[id];
}

structures::IArray<PointMass*> const* ThorMagniPedScene::get_point_masses() const
{
    return id_mass_dict.get_values();
}

//RectRigidBodyEnv* ThorMagniPedScene::get_env()
//{
//    return &scene_env;
//}

}
}
}
}

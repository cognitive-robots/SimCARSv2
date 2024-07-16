#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agents/ped_scene_interface.hpp>

#include <rapidcsv.h>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace thor_magni
{

class ThorMagniPedScene : public IPedScene
{
    temporal::Time min_time;
    temporal::Time max_time;

    //RectRigidBodyEnv scene_env;
    structures::stl::STLDictionary<uint32_t, PointMass*> id_mass_dict;

public:
    ThorMagniPedScene(std::string const &scene_path_str);
    ThorMagniPedScene(rapidcsv::Document const &scene_doc);

    ~ThorMagniPedScene() override;

    temporal::Duration get_time_step_size() const override;
    temporal::Time get_min_time() const override;
    temporal::Time get_max_time() const override;
    PointMass* get_point_mass(uint32_t id) const override;
    structures::IArray<PointMass*> const* get_point_masses() const override;

    //RectRigidBodyEnv* get_env() override;
};

}
}
}
}

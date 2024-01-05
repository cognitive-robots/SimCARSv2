#pragma once

#include <ori/simcars/agents/fwd_car_scene_interface.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

#include <rapidcsv.h>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace highd
{

class HighDFWDCarScene : public IFWDCarScene
{
    temporal::Time min_time;
    temporal::Time max_time;

    RectRigidBodyEnv scene_env;
    structures::stl::STLDictionary<uint32_t, FWDCar*> id_car_dict;

public:
    HighDFWDCarScene(std::string const &tracks_meta_path_str, std::string const &tracks_path_str);
    HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc, rapidcsv::Document const &tracks_doc);

    ~HighDFWDCarScene() override;

    temporal::Duration get_time_step_size() const override;
    temporal::Time get_min_time() const override;
    temporal::Time get_max_time() const override;
};

}
}
}
}

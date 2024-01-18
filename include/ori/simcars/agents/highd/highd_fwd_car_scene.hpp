#pragma once

#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agents/fwd_car_scene_interface.hpp>

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
    structures::stl::STLDictionary<uint32_t, ControlFWDCar*> id_control_dict;
    structures::stl::STLDictionary<uint32_t, PlanFWDCar*> id_plan_dict;
public:
    HighDFWDCarScene(std::string const &tracks_meta_path_str, std::string const &tracks_path_str,
                     map::IMap const *map = nullptr);
    HighDFWDCarScene(std::string const &tracks_meta_path_str, std::string const &tracks_path_str,
                     size_t start_frame, size_t end_frame, map::IMap const *map = nullptr);
    HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc,
                     rapidcsv::Document const &tracks_doc, map::IMap const *map = nullptr);
    HighDFWDCarScene(rapidcsv::Document const &tracks_meta_doc,
                     rapidcsv::Document const &tracks_doc, size_t start_frame, size_t end_frame,
                     map::IMap const *map = nullptr);

    ~HighDFWDCarScene() override;

    temporal::Duration get_time_step_size() const override;
    temporal::Time get_min_time() const override;
    temporal::Time get_max_time() const override;
    FWDCar* get_fwd_car(uint32_t id) const override;
    ControlFWDCar* get_control_fwd_car(uint32_t id) const override;
    PlanFWDCar* get_plan_fwd_car(uint32_t id) const override;
    structures::IArray<FWDCar*> const* get_fwd_cars() const override;
    structures::IArray<ControlFWDCar*> const* get_control_fwd_cars() const override;
    structures::IArray<PlanFWDCar*> const* get_plan_fwd_cars() const override;
};

}
}
}
}

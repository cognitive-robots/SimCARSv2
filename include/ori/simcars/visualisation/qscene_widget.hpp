#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/visualisation/qsfml_canvas_abstract.hpp>

#include <memory>
#include <mutex>

namespace ori
{
namespace simcars
{
namespace visualisation
{

class QSceneWidget : public AQSFMLCanvas
{
    bool text_enabled;
    sf::Font text_font;

    std::shared_ptr<const agent::IScene> scene;

    FP_DATA_TYPE realtime_factor;
    FP_DATA_TYPE pixels_per_metre;

    bool focused;
    geometry::Vec focal_position;
    std::shared_ptr<const structures::IArray<uint32_t>> focal_ego_agents, focal_non_ego_agents;

    temporal::Time earliest_time, latest_time, current_time;

    temporal::Time last_time, last_realtime;

    bool update_required;

    mutable std::recursive_mutex mutex;

protected:
    std::shared_ptr<const geometry::TrigBuff> trig_buff;

    structures::stl::STLStackArray<std::shared_ptr<const sf::Drawable>> render_stack;

    void on_init() override;
    void on_update() override;

    virtual void add_agent_to_render_stack(std::shared_ptr<const agent::IAgent> agent);
    virtual void add_scene_to_render_stack();
    virtual void populate_render_stack();

public:
    QSceneWidget(std::shared_ptr<const agent::IScene> scene, QWidget* parent, const QPoint& position, const QSize& size,
                 FP_DATA_TYPE frame_rate = 30.0f, FP_DATA_TYPE realtime_factor = 1.0f, FP_DATA_TYPE pixels_per_metre = 10.0f);

    FP_DATA_TYPE get_pixels_per_metre() const;
    const geometry::Vec& get_focal_position() const;
    std::shared_ptr<const structures::IArray<uint32_t>> get_focal_ego_agents() const;
    std::shared_ptr<const structures::IArray<uint32_t>> get_focal_non_ego_agents() const;
    temporal::Time get_time() const;

    void set_focal_ego_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_ego_agents);
    void set_focal_non_ego_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_non_ego_agents);
    void set_focal_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_ego_agents,
                          std::shared_ptr<const structures::IArray<uint32_t>> focal_non_ego_agents);
    void set_time(temporal::Time time);

public slots:
    void tick_forwards();
    void tick_backwards();
};

}
}
}

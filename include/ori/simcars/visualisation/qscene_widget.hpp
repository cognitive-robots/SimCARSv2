#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/visualisation/qsfml_canvas_abstract.hpp>

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

    agent::IScene const *scene;

    FP_DATA_TYPE realtime_factor;
    FP_DATA_TYPE pixels_per_metre;

    bool focused;
    geometry::Vec focal_position;
    structures::IArray<std::string> const *focal_entities;

    temporal::Time current_time;

    temporal::Time last_time, last_realtime;

    bool update_required;

    mutable std::recursive_mutex mutex;

protected:
    geometry::TrigBuff const *trig_buff;

    structures::stl::STLStackArray<sf::Drawable const*> render_stack;

    void on_init() override;
    void on_update() override;

    virtual void add_vehicle_to_render_stack(agent::IEntity const *vehicle);
    virtual void add_scene_to_render_stack();
    virtual void populate_render_stack();

public:
    QSceneWidget(agent::IScene const *scene, QWidget *parent, QPoint const &position, QSize const &size,
                 FP_DATA_TYPE frame_rate = 30.0f, FP_DATA_TYPE realtime_factor = 1.0f, FP_DATA_TYPE pixels_per_metre = 10.0f);

    FP_DATA_TYPE get_pixels_per_metre() const;
    geometry::Vec const& get_focal_position() const;
    structures::IArray<std::string> const* get_focal_entities() const;
    temporal::Time get_time() const;

    void set_focal_entities(structures::IArray<std::string> const *focal_entities);
    void set_time(temporal::Time time);

public slots:
    void tick_forwards();
    void tick_backwards();
};

}
}
}

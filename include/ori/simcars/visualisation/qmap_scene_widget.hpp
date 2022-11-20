#pragma once

#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/map/traffic_light_interface.hpp>
#include <ori/simcars/visualisation/utils.hpp>
#include <ori/simcars/visualisation/qscene_widget.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

template <typename T_id>
class QMapSceneWidget : public QSceneWidget
{
    map::IMap<T_id> const *map;

    FP_DATA_TYPE single_point_map_object_size;

protected:
    void populate_render_stack() override
    {
        add_map_to_render_stack();
        this->add_scene_to_render_stack();
    }

    virtual void add_lane_to_render_stack(map::ILane<T_id> const *lane)
    {
        structures::IArray<geometry::Tri> const *tris = lane->get_tris();
        size_t i, j;
        for (i = 0; i < tris->count(); ++i)
        {
            sf::ConvexShape *polygon = new sf::ConvexShape;
            polygon->setPointCount(3);
            for (j = 0; j < 3; ++j)
            {
                polygon->setPoint(j, to_sfml_vec(get_pixels_per_metre() * (*tris)[i][j], false, true));
            }

            T_id id = lane->get_id();
            size_t id_hash = std::hash<T_id>()(id);

            float h = id_hash % 360;
            float s = 0.5f;
            float v = 1.0f;

            float c = v * s;
            float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
            float m = v - c;

            uint8_t r, g, b;
            if (0.0f <= h && h < 60.0f)
            {
                r = 255 * (c + m);
                g = 255 * (x + m);
                b = 255 * m;
            }
            else if (60.0f <= h && h < 120.0f)
            {
                r = 255 * (x + m);
                g = 255 * (c + m);
                b = 255 * m;
            }
            else if (120.0f <= h && h < 180.0f)
            {
                r = 255 * m;
                g = 255 * (c + m);
                b = 255 * (x + m);
            }
            else if (180.0f <= h && h < 240.0f)
            {
                r = 255 * m;
                g = 255 * (x + m);
                b = 255 * (c + m);
            }
            else if (240.0f <= h && h < 300.0f)
            {
                r = 255 * (x + m);
                g = 255 * m;
                b = 255 * (c + m);
            }
            else if (300.0f <= h && h < 360.0f)
            {
                r = 255 * (c + m);
                g = 255 * m;
                b = 255 * (x + m);
            }

            polygon->setFillColor(sf::Color(r, g, b, 128));

            render_stack.push_back(polygon);
        }
    }
    virtual void add_traffic_light_to_render_stack(map::ITrafficLight<T_id> const *traffic_light)
    {
        sf::CircleShape *circle = new sf::CircleShape(get_pixels_per_metre() * single_point_map_object_size);
        map::ITrafficLightStateHolder::State const *traffic_light_state = traffic_light->get_state(this->get_time());
        circle->setPosition(to_sfml_vec(get_pixels_per_metre() * traffic_light->get_position(), false, true));
        circle->setFillColor(to_sfml_colour(traffic_light_state->active_face));
        render_stack.push_back(circle);
    }
    virtual void add_map_to_render_stack()
    {
        geometry::Vec const &point = this->get_focal_position() / this->get_pixels_per_metre();
        FP_DATA_TYPE distance = std::max(this->width() / this->get_pixels_per_metre(),
                                         this->height() / this->get_pixels_per_metre());

        map::ILaneArray<T_id> const *lanes_in_focus = map->get_lanes_in_range(point, distance);

        size_t i;
        for (i = 0; i < lanes_in_focus->count(); ++i)
        {
            add_lane_to_render_stack((*lanes_in_focus)[i]);
        }

        map::ITrafficLightArray<T_id> const *traffic_lights_in_focus =
                map->get_traffic_lights_in_range(point, distance);

        for (i = 0; i < traffic_lights_in_focus->count(); ++i)
        {
            add_traffic_light_to_render_stack((*traffic_lights_in_focus)[i]);
        }
    }

public:
    QMapSceneWidget(map::IMap<T_id> const *map, agent::IScene const *scene, QWidget *parent,
                    QPoint const &position, QSize const &size, FP_DATA_TYPE single_point_map_object_size = 1.0f, FP_DATA_TYPE frame_rate = 30.0f,
                    FP_DATA_TYPE realtime_factor = 1.0f, FP_DATA_TYPE pixels_per_metre = 10.0f)
        : QSceneWidget(scene, parent, position, size, frame_rate, realtime_factor, pixels_per_metre), map(map),
          single_point_map_object_size(single_point_map_object_size) {}
};

}
}
}

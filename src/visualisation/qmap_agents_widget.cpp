
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>


namespace ori
{
namespace simcars
{
namespace visualisation
{

void QMapAgentsWidget::add_lane_to_render_stack(map::ILane const *lane)
{
    structures::IArray<geometry::Tri> const *tris = lane->get_tris();
    size_t i, j;
    for (i = 0; i < tris->count(); ++i)
    {
        uint64_t id = lane->get_id();

        sf::Color lane_colour;
        if (id_to_colour_dict.contains(id))
        {
            lane_colour = id_to_colour_dict[id];
        }
        else
        {
            float h = hue_generator(randomness_generator);
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

            lane_colour = sf::Color(r, g, b, 128);

            id_to_colour_dict.update(id, lane_colour);
        }

        sf::VertexArray *lane_shape = new sf::VertexArray(sf::PrimitiveType::Triangles,
                                                          3 * tris->count());

        for (j = 0; j < 3; ++j)
        {
            geometry::Vec tri_point = (*tris)[i][j];
            (*lane_shape)[i * 3 + j].position = sf::Vector2f(get_pixels_per_metre() * tri_point.x(),
                                                       -get_pixels_per_metre() * tri_point.y());
            (*lane_shape)[i * 3 + j].color = lane_colour;
        }

        add_to_render_stack(lane_shape);
    }
}

void QMapAgentsWidget::populate_render_stack()
{
    add_map_to_render_stack();
    add_agents_to_render_stack();
}

void QMapAgentsWidget::add_map_to_render_stack()
{
    geometry::Vec const &point = get_focal_position();
    FP_DATA_TYPE distance = std::max(width() / get_pixels_per_metre(),
                                     height() / get_pixels_per_metre());

    structures::IArray<map::ILane const*> const *lanes_in_view =
            map->get_lanes_in_range(point, distance);

    for (size_t i = 0; i < lanes_in_view->count(); ++i)
    {
        add_lane_to_render_stack((*lanes_in_view)[i]);
    }

    delete lanes_in_view;
}

QMapAgentsWidget::QMapAgentsWidget(map::IMap const *map, QWidget *parent, QPoint const &position,
                                   QSize const &size, temporal::Time start_time,
                                   temporal::Time end_time,
                                   std::chrono::milliseconds frame_interval,
                                   FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre,
                                   FocusMode focus_mode) :
    QAgentsWidget(parent, position, size, start_time, end_time, frame_interval, realtime_factor,
                  pixels_per_metre), map(map) {}

}
}
}

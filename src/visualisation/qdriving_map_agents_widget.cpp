
#include <ori/simcars/visualisation/qdriving_map_agents_widget.hpp>


namespace ori
{
namespace simcars
{
namespace visualisation
{

void QDrivingMapAgentsWidget::add_lane_to_render_stack(map::ILane const *lane)
{
    sf::Color road_color(64, 64, 64);
    sf::Color markings_color(128, 128, 128);

    geometry::Vecs left_boundary = lane->get_left_boundary();
    geometry::Vecs right_boundary = lane->get_right_boundary();

    size_t triangle_count = left_boundary.cols() + right_boundary.cols() - 2;

    sf::VertexArray *lane_shape = new sf::VertexArray(sf::PrimitiveType::Triangles,
                                                      3 * triangle_count);

    size_t i = 0, j = 0, k = 0;
    while (i < left_boundary.cols() - 1 || j < right_boundary.cols() - 1)
    {
        (*lane_shape)[3 * k].position = sf::Vector2f(get_pixels_per_metre() * left_boundary(0, i),
                                                 -get_pixels_per_metre() * left_boundary(1, i));
        (*lane_shape)[3 * k].color = road_color;

        (*lane_shape)[3 * k + 1].position = sf::Vector2f(get_pixels_per_metre() * right_boundary(0, j),
                                                 -get_pixels_per_metre() * right_boundary(1, j));
        (*lane_shape)[3 * k + 1].color = road_color;

        if (i >= left_boundary.cols() - 1)
        {
            ++j;

            (*lane_shape)[3 * k + 2].position = sf::Vector2f(get_pixels_per_metre() * right_boundary(0, j),
                                                     -get_pixels_per_metre() * right_boundary(1, j));
            (*lane_shape)[3 * k + 2].color = road_color;
        }
        else if (j >= right_boundary.cols() - 1)
        {
            ++i;

            (*lane_shape)[3 * k + 2].position = sf::Vector2f(get_pixels_per_metre() * left_boundary(0, i),
                                                     -get_pixels_per_metre() * left_boundary(1, i));
            (*lane_shape)[3 * k + 2].color = road_color;
        }
        else
        {
            if (i <= j)
            {
                ++i;

                (*lane_shape)[3 * k + 2].position = sf::Vector2f(get_pixels_per_metre() * left_boundary(0, i),
                                                         -get_pixels_per_metre() * left_boundary(1, i));
                (*lane_shape)[3 * k + 2].color = road_color;
            }
            else
            {
                ++j;

                (*lane_shape)[3 * k + 2].position = sf::Vector2f(get_pixels_per_metre() * right_boundary(0, j),
                                                         -get_pixels_per_metre() * right_boundary(1, j));
                (*lane_shape)[3 * k + 2].color = road_color;
            }
        }

        ++k;
    }

    sf::VertexArray *boundary_shape = new sf::VertexArray(sf::PrimitiveType::LineStrip,
                                                               left_boundary.cols() +
                                                               right_boundary.cols() + 1);

    for (i = 0; i < left_boundary.cols(); ++i)
    {
        (*boundary_shape)[i].position = sf::Vector2f(get_pixels_per_metre() * left_boundary(0, i),
                                                     -get_pixels_per_metre() * left_boundary(1, i));
        (*boundary_shape)[i].color = markings_color;
    }

    for (i = 0; i < right_boundary.cols(); ++i)
    {
        (*boundary_shape)[left_boundary.cols() + i].position =
                sf::Vector2f(get_pixels_per_metre() * right_boundary(0, (right_boundary.cols() - 1) - i),
                             -get_pixels_per_metre() * right_boundary(1, (right_boundary.cols() - 1) - i));
        (*boundary_shape)[left_boundary.cols() + i].color = markings_color;
    }

    (*boundary_shape)[left_boundary.cols() + right_boundary.cols()].position =
            sf::Vector2f(get_pixels_per_metre() * left_boundary(0, 0),
                         -get_pixels_per_metre() * left_boundary(1, 0));
    (*boundary_shape)[left_boundary.cols() + right_boundary.cols()].color = markings_color;

    add_to_render_stack(lane_shape);
    add_to_render_stack(boundary_shape);
}

void QDrivingMapAgentsWidget::populate_render_stack()
{
    add_map_to_render_stack();
    add_agents_to_render_stack();
}

void QDrivingMapAgentsWidget::add_map_to_render_stack()
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

QDrivingMapAgentsWidget::QDrivingMapAgentsWidget(map::IDrivingMap const *map, QWidget *parent, QPoint const &position,
                                   QSize const &size, temporal::Time start_time,
                                   temporal::Time end_time,
                                   std::chrono::milliseconds frame_interval,
                                   FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre,
                                   FocusMode focus_mode) :
    QDrivingAgentsWidget(parent, position, size, start_time, end_time, frame_interval, realtime_factor,
                  pixels_per_metre),
    map(map) {}

}
}
}

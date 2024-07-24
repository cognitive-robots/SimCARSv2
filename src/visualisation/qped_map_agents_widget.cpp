
#include <ori/simcars/visualisation/qped_map_agents_widget.hpp>


namespace ori
{
namespace simcars
{
namespace visualisation
{

void QPedMapAgentsWidget::populate_render_stack()
{
    add_map_to_render_stack();
    add_agents_to_render_stack();
}

void QPedMapAgentsWidget::add_map_to_render_stack()
{
    sf::Sprite *sprite = new sf::Sprite();
    sprite->setTexture(map->get_texture(), true);
    FP_DATA_TYPE texture_scale = map->get_texture_scale();
    geometry::Vec const &texture_offset = map->get_texture_offset();
    sprite->move(get_pixels_per_metre() * texture_scale * texture_offset.x(),
                 get_pixels_per_metre() * texture_scale * texture_offset.y());
    sprite->scale(get_pixels_per_metre() * texture_scale, get_pixels_per_metre() * texture_scale);
    add_to_render_stack(sprite);


    structures::IArray<geometry::VecPair>* const edge_array = map->get_edges();

    sf::VertexArray *edge_lines = new sf::VertexArray(sf::Lines, edge_array->count() * 2);

    size_t i;
    for (i = 0; i < edge_array->count(); ++i)
    {
        geometry::VecPair const &edge = (*edge_array)[i];
        (*edge_lines)[2 * i].position = sf::Vector2f(get_pixels_per_metre() * edge.first.x(),
                                                     -get_pixels_per_metre() * edge.first.y());
        (*edge_lines)[2 * i].color = sf::Color::Red;
        (*edge_lines)[2 * i + 1].position = sf::Vector2f(get_pixels_per_metre() * edge.second.x(),
                                                         -get_pixels_per_metre() * edge.second.y());
        (*edge_lines)[2 * i + 1].color = sf::Color::Red;
    }

    add_to_render_stack(edge_lines);

    delete edge_array;


    FP_DATA_TYPE const node_radius = 0.1;

    structures::IArray<map::INode const*>* const node_array = map->get_nodes();

    for (i = 0; i < node_array->count(); ++i)
    {
        map::INode const* const node = (*node_array)[i];
        geometry::Vec const &centroid = node->get_centroid();

        sf::CircleShape *node_shape = new sf::CircleShape(get_pixels_per_metre() * node_radius, 8);

        node_shape->setFillColor(sf::Color::Red);

        node_shape->setPosition(sf::Vector2f(get_pixels_per_metre() * centroid.x() -
                                             get_pixels_per_metre() * node_radius,
                                             -get_pixels_per_metre() * centroid.y() -
                                             get_pixels_per_metre() * node_radius));

        add_to_render_stack(node_shape);
    }

    delete node_array;


    structures::IArray<map::INode const*>* const goal_node_array = map->get_goal_nodes();

    for (i = 0; i < goal_node_array->count(); ++i)
    {
        map::INode const* const goal_node = (*goal_node_array)[i];
        geometry::Vec const &centroid = goal_node->get_centroid();

        sf::CircleShape *goal_node_shape = new sf::CircleShape(get_pixels_per_metre() * node_radius, 8);

        goal_node_shape->setFillColor(sf::Color::Blue);

        goal_node_shape->setPosition(sf::Vector2f(get_pixels_per_metre() * centroid.x() -
                                                  get_pixels_per_metre() * node_radius,
                                                  -get_pixels_per_metre() * centroid.y() -
                                                  get_pixels_per_metre() * node_radius));

        add_to_render_stack(goal_node_shape);
    }

    delete goal_node_array;
}

QPedMapAgentsWidget::QPedMapAgentsWidget(map::IPedMap const *map, QWidget *parent,
                                         QPoint const &position, QSize const &size,
                                         temporal::Time start_time, temporal::Time end_time,
                                         std::chrono::milliseconds frame_interval,
                                         FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre,
                                         FocusMode focus_mode) :
    QPedAgentsWidget(parent, position, size, start_time, end_time, frame_interval, realtime_factor,
                  pixels_per_metre, focus_mode),
    map(map) {}

}
}
}


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
    sf::Sprite *sprite = map->get_sprite();
    add_to_render_stack(sprite);
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

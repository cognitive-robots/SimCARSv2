#pragma once

#include <ori/simcars/map/ped_map_interface.hpp>
#include <ori/simcars/visualisation/qped_agents_widget.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

class QPedMapAgentsWidget : public QPedAgentsWidget
{
    map::IPedMap const *map;

protected:
    void populate_render_stack() override;

    void add_map_to_render_stack();

public:
    QPedMapAgentsWidget(map::IPedMap const *map, QWidget *parent, QPoint const &position,
                        QSize const &size, temporal::Time start_time, temporal::Time end_time,
                        std::chrono::milliseconds frame_interval = std::chrono::milliseconds(100),
                        FP_DATA_TYPE realtime_factor = 1.0f, FP_DATA_TYPE pixels_per_metre = 2.0,
                        FocusMode focus_mode = FocusMode::ALL_AGENTS);
};

}
}
}

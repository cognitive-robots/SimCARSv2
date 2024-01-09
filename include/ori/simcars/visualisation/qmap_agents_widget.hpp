#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/map/lane_interface.hpp>
#include <ori/simcars/visualisation/qagents_widget.hpp>

#include <random>

namespace ori
{
namespace simcars
{
namespace visualisation
{

class QMapAgentsWidget : public QAgentsWidget
{
    map::IMap const *map;

    void add_lane_to_render_stack(map::ILane const *lane);

protected:
    void populate_render_stack() override;

    void add_map_to_render_stack();

public:
    QMapAgentsWidget(map::IMap const *map, QWidget *parent, QPoint const &position,
                     QSize const &size, temporal::Time start_time, temporal::Time end_time,
                     std::chrono::milliseconds frame_interval = std::chrono::milliseconds(100),
                     FP_DATA_TYPE realtime_factor = 1.0f, FP_DATA_TYPE pixels_per_metre = 2.0,
                     FocusMode focus_mode = FocusMode::ALL_AGENTS);
};

}
}
}

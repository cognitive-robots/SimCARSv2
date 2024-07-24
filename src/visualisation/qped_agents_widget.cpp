
#include <ori/simcars/visualisation/qped_agents_widget.hpp>

#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/variable_context.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace visualisation
{

void QPedAgentsWidget::add_agent_to_render_stack(agents::PointMass *agent)
{
    geometry::Vec agent_pos;
    bool res = agent->get_pos_variable()->get_value(agent_pos);

    if (!res)
    {
        return;
    }

    uint64_t id;
    res = agent->get_id_variable()->get_value(id);

    sf::Color agent_colour;
    if (res && id_colour_dict.contains(id))
    {
        agent_colour = id_colour_dict[id];
    }
    else
    {
        agent_colour = sf::Color::Green;
    }

    FP_DATA_TYPE const agent_radius = 0.175;

    sf::CircleShape *agent_shape = new sf::CircleShape(get_pixels_per_metre() * agent_radius, 8);

    agent_shape->setFillColor(agent_colour);

    agent_shape->setPosition(sf::Vector2f(get_pixels_per_metre() * agent_pos.x() -
                                          get_pixels_per_metre() * agent_radius,
                                          -get_pixels_per_metre() * agent_pos.y() -
                                          get_pixels_per_metre() * agent_radius));

    if (focus_mode == FocusMode::ALL_AGENTS ||
            (focus_mode == FocusMode::FOCAL_AGENTS && res && focal_agent_ids->contains(id)))
    {
        focal_position += agent_pos;
        ++focal_agent_count;
    }

    add_to_render_stack(agent_shape);

    /*
    if (text_enabled)
    {
        sf::Text *agent_text = new sf::Text(std::to_string(id), text_font, 16);
        agent_text->setPosition(sf::Vector2f(get_pixels_per_metre() * agent_pos.x(),
                                             -get_pixels_per_metre() * agent_pos.y()));
        add_to_render_stack(agent_text);
    }
    */
}

void QPedAgentsWidget::on_init()
{
    // TODO: Make this non-OS specific
    if(!text_font.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf"))
    {
        text_enabled = false;
    }

    //causal::VariableContext::set_time_step_size(
    //            std::chrono::duration_cast<temporal::Duration>(frame_interval));

    on_update();
}

void QPedAgentsWidget::on_update()
{
    if (data_mutex.try_lock())
    {
        std::lock_guard<std::recursive_mutex> const lock(data_mutex, std::adopt_lock);

        tick_forwards();

        if (update_required)
        {
            causal::VariableContext::set_current_time(current_time);

            for (size_t i = 0; i < render_stack.count(); ++i)
            {
                delete render_stack[i];
            }
            render_stack.clear();

            populate_render_stack();

            sf::Vector2f centre = sf::Vector2f(get_pixels_per_metre() * focal_position.x(),
                                               -get_pixels_per_metre() * focal_position.y());
            sf::Vector2f dims = sf::Vector2f(width(), height());
            sf::View view(centre, dims);
            setView(view);

            clear(sf::Color(0, 0, 0));
            for (size_t i = 0; i < render_stack.count(); ++i)
            {
                draw(*(render_stack[i]));
            }

            sf::Text timer_text(
                        std::to_string(
                            std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                current_time.time_since_epoch()).count()) + " s",
                        text_font, 16);
            timer_text.setPosition(centre.x - 0.5 * width(), centre.y - 0.5 * height());
            draw(timer_text);

            update_required = false;
        }
    }
}

void QPedAgentsWidget::add_to_render_stack(sf::Drawable const *drawable)
{
    render_stack.push_back(drawable);
}

void QPedAgentsWidget::add_agents_to_render_stack()
{
    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position = geometry::Vec::Zero();
    }
    focal_agent_count = 0;

    structures::IArray<agents::PointMass*> const *agent_array = get_array();
    for (size_t i = 0; i < agent_array->count(); ++i)
    {
        agents::PointMass *agent = (*agent_array)[i];
        add_agent_to_render_stack(agent);
    }

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position /= focal_agent_count;
    }
}

void QPedAgentsWidget::populate_render_stack()
{
    add_agents_to_render_stack();
}

QPedAgentsWidget::QPedAgentsWidget(
        QWidget *parent, QPoint const &position, QSize const &size, temporal::Time start_time,
        temporal::Time end_time, std::chrono::milliseconds frame_interval,
        FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre, FocusMode focus_mode) :
    AQSFMLCanvas(parent, position, size, frame_interval), text_enabled(true),
    frame_interval(frame_interval), realtime_factor(realtime_factor),
    pixels_per_metre(pixels_per_metre), focus_mode(focus_mode), focal_agent_ids(nullptr),
    start_time(start_time), end_time(end_time), current_time(start_time),
    last_realtime(std::chrono::time_point<std::chrono::steady_clock>::min()),
    update_required(false) {}

QPedAgentsWidget::~QPedAgentsWidget()
{
    for (size_t i = 0; i < render_stack.count(); ++i)
    {
        delete render_stack[i];
    }
}

FP_DATA_TYPE QPedAgentsWidget::get_realtime_factor() const
{
    return realtime_factor;
}

FP_DATA_TYPE QPedAgentsWidget::get_pixels_per_metre() const
{
    return pixels_per_metre;
}

QPedAgentsWidget::FocusMode QPedAgentsWidget::get_focus_mode() const
{
    return focus_mode;
}

geometry::Vec const& QPedAgentsWidget::get_focal_position() const
{
    return focal_position;
}

structures::IArray<uint64_t> const* QPedAgentsWidget::get_focal_agent_ids() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return focal_agent_ids;
}

temporal::Time QPedAgentsWidget::get_time() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return current_time;
}

void QPedAgentsWidget::set_realtime_factor(FP_DATA_TYPE realtime_factor)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    this->realtime_factor = realtime_factor;
}

void QPedAgentsWidget::set_pixels_per_metre(FP_DATA_TYPE pixels_per_metre)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->pixels_per_metre != pixels_per_metre)
    {
        this->pixels_per_metre = pixels_per_metre;
        update_required = true;
    }
}

void QPedAgentsWidget::set_focus_mode(FocusMode focus_mode)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->focus_mode != focus_mode)
    {
        this->focus_mode = focus_mode;
        update_required = true;
    }
}

void QPedAgentsWidget::set_focal_position(geometry::Vec const &focal_position)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (focus_mode != FocusMode::FIXED || this->focal_position != focal_position)
    {
        focus_mode = FocusMode::FIXED;
        this->focal_position = focal_position;
        update_required = true;
    }
}

void QPedAgentsWidget::set_focal_agent_ids(structures::IArray<uint64_t> const *focal_agent_ids)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->focal_agent_ids != focal_agent_ids)
    {
        if (this->focal_agent_ids != nullptr)
        {
            delete this->focal_agent_ids;
        }

        this->focal_agent_ids = focal_agent_ids;

        // TODO: Possibly add a check to see if an update is actually required?
        if (focus_mode == FocusMode::FOCAL_AGENTS)
        {
            update_required = true;
        }
    }
}

void QPedAgentsWidget::set_agent_colour(uint64_t agent_id, sf::Color colour)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (id_colour_dict.contains(agent_id) && id_colour_dict[agent_id] != colour)
    {
        update_required = true;
    }

    id_colour_dict.update(agent_id, colour);
}

void QPedAgentsWidget::set_time(temporal::Time time)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (time < start_time)
    {
        current_time = start_time;
    }
    else if (time > end_time)
    {
        current_time = end_time;
    }
    else
    {
        current_time = time;
    }

    update_required = true;
}

void QPedAgentsWidget::tick_forwards()
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    std::chrono::time_point<std::chrono::steady_clock> current_realtime =
            std::chrono::steady_clock::now();

    if (last_realtime != std::chrono::time_point<std::chrono::steady_clock>::min())
    {
        std::chrono::steady_clock::duration realtime_diff =
                current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        if (current_time + time_diff < end_time)
        {
            current_time += time_diff;
            update_required = true;
        }
        else if (current_time != end_time)
        {
            current_time = end_time;
            update_required = true;
        }
    }

    last_realtime = current_realtime;
}

void QPedAgentsWidget::tick_backwards()
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    std::chrono::time_point<std::chrono::steady_clock> current_realtime =
            std::chrono::steady_clock::now();

    if (last_realtime != std::chrono::time_point<std::chrono::steady_clock>::min())
    {
        std::chrono::steady_clock::duration realtime_diff =
                current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        if (current_time - time_diff > start_time)
        {
            current_time -= time_diff;
            update_required = true;
        }
        else if (current_time != start_time)
        {
            current_time = start_time;
            update_required = true;
        }
    }

    last_realtime = current_realtime;
}

}
}
}

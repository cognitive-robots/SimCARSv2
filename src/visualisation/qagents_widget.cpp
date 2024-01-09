
#include <ori/simcars/visualisation/qagents_widget.hpp>

#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

void QAgentsWidget::add_agent_to_render_stack(agents::FWDCar *agent)
{
    geometry::ORect agent_rect;
    bool res = agent->get_rect_variable()->get_value(agent_rect);

    if (!res)
    {
        return;
    }

    sf::VertexArray *agent_shape = new sf::VertexArray(sf::PrimitiveType::TriangleStrip, 4);

    geometry::Vec rect_point = agent_rect[0];
    (*agent_shape)[0].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[0].color = sf::Color::Green;

    rect_point = agent_rect[3];
    (*agent_shape)[1].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[1].color = sf::Color::Green;

    rect_point = agent_rect[1];
    (*agent_shape)[2].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[2].color = sf::Color::Green;

    rect_point = agent_rect[2];
    (*agent_shape)[3].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[3].color = sf::Color::Green;

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position += agent_rect.get_origin();
        focal_agent_count++;
    }

    add_to_render_stack(agent_shape);
}

void QAgentsWidget::on_init()
{
    causal::VariableContext::set_time_step_size(
                std::chrono::duration_cast<temporal::Duration>(frame_interval));

    on_update();
}

void QAgentsWidget::on_update()
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

            sf::View view(sf::Vector2f(get_pixels_per_metre() * focal_position.x(),
                                       -get_pixels_per_metre() * focal_position.y()),
                          sf::Vector2f(width(), height()));
            setView(view);

            clear(sf::Color(0, 0, 0));
            for (size_t i = 0; i < render_stack.count(); ++i)
            {
                draw(*(render_stack[i]));
            }

            update_required = false;
        }
    }
}

void QAgentsWidget::add_to_render_stack(sf::Drawable const *drawable)
{
    render_stack.push_back(drawable);
}

void QAgentsWidget::add_agents_to_render_stack()
{
    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position = geometry::Vec::Zero();
    }
    focal_agent_count = 0;

    structures::IArray<agents::FWDCar*> const *agent_array = get_array();
    for (size_t i = 0; i < agent_array->count(); ++i)
    {
        agents::FWDCar *agent = (*agent_array)[i];
        add_agent_to_render_stack(agent);
    }

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position /= focal_agent_count;
    }
}

void QAgentsWidget::populate_render_stack()
{
    add_agents_to_render_stack();
}

QAgentsWidget::QAgentsWidget(
        QWidget *parent, QPoint const &position, QSize const &size, temporal::Time start_time,
        temporal::Time end_time, std::chrono::milliseconds frame_interval,
        FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre, FocusMode focus_mode) :
    AQSFMLCanvas(parent, position, size, frame_interval), frame_interval(frame_interval),
    realtime_factor(realtime_factor), pixels_per_metre(pixels_per_metre), focus_mode(focus_mode),
    start_time(start_time), end_time(end_time), current_time(start_time),
    last_realtime(std::chrono::time_point<std::chrono::steady_clock>::min()),
    update_required(false) {}

QAgentsWidget::~QAgentsWidget()
{
    for (size_t i = 0; i < render_stack.count(); ++i)
    {
        delete render_stack[i];
    }
}

FP_DATA_TYPE QAgentsWidget::get_realtime_factor() const
{
    return realtime_factor;
}

FP_DATA_TYPE QAgentsWidget::get_pixels_per_metre() const
{
    return pixels_per_metre;
}

QAgentsWidget::FocusMode QAgentsWidget::get_focus_mode() const
{
    return focus_mode;
}

geometry::Vec const& QAgentsWidget::get_focal_position() const
{
    return focal_position;
}

/*
structures::IArray<std::string> const* QAgentsWidget::get_focal_entities() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return focal_entities;
}
*/

temporal::Time QAgentsWidget::get_time() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return current_time;
}

void QAgentsWidget::set_realtime_factor(FP_DATA_TYPE realtime_factor)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    this->realtime_factor = realtime_factor;
}

void QAgentsWidget::set_pixels_per_metre(FP_DATA_TYPE pixels_per_metre)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->pixels_per_metre != pixels_per_metre)
    {
        this->pixels_per_metre = pixels_per_metre;
        update_required = true;
    }
}

void QAgentsWidget::set_focus_mode(FocusMode focus_mode)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->focus_mode != focus_mode)
    {
        this->focus_mode = focus_mode;
        update_required = true;
    }
}

void QAgentsWidget::set_focal_position(geometry::Vec const &focal_position)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (focus_mode != FocusMode::FIXED || this->focal_position != focal_position)
    {
        focus_mode = FocusMode::FIXED;
        this->focal_position = focal_position;
        update_required = true;
    }
}

/*
void QAgentsWidget::set_focal_entities(structures::IArray<std::string> const *focal_entities)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->focal_entities != focal_entities)
    {
        delete this->focal_entities;
        this->focal_entities = focal_entities;

        // TODO: Possibly add a check to see if an update is actually required?
        if (focus_mode == FocusMode::FOCAL_AGENTS)
        {
            update_required = true;
        }
    }
}
*/

void QAgentsWidget::set_time(temporal::Time time)
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

void QAgentsWidget::tick_forwards()
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

void QAgentsWidget::tick_backwards()
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

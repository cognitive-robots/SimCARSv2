
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
    /*
    agent::IConstant<bool> const *ego_constant = driving_agent->get_ego_constant();
    agent::IConstant<uint32_t> const *id_constant = driving_agent->get_id_constant();
    agent::IConstant<agent::DrivingAgentClass> const *driving_agent_class_constant =
            driving_agent->get_driving_agent_class_constant();
    agent::IConstant<FP_DATA_TYPE> const *bb_length_constant =
            driving_agent->get_bb_length_constant();
    agent::IConstant<FP_DATA_TYPE> const *bb_width_constant =
            driving_agent->get_bb_width_constant();

    agent::IVariable<geometry::Vec> const *position_variable =
            driving_agent->get_position_variable();
    agent::IVariable<FP_DATA_TYPE> const *rotation_variable =
            driving_agent->get_rotation_variable();

    temporal::Time current_time = this->get_time();
    agent::ViewReadOnlyDrivingAgentState state(
                dynamic_cast<agent::IDrivingAgent const*>(vehicle), current_time);

    sf::RectangleShape *rectangle = nullptr;
    sf::CircleShape *circle = nullptr;
    sf::Text *text = nullptr;

    geometry::Vec position;
    if (!position_variable->get_value(current_time, position))
    {
        return;
    }

    FP_DATA_TYPE rotation;
    if (!rotation_variable->get_value(current_time, rotation))
    {
        return;
    }

    sf::Vector2f agent_base_shape_position = to_sfml_vec(get_pixels_per_metre() * position, false, this->get_flip_y());

    FP_DATA_TYPE agent_rectangle_length = get_pixels_per_metre() * bb_length_constant->get_value();
    FP_DATA_TYPE agent_rectangle_width = get_pixels_per_metre() * bb_width_constant->get_value();
    FP_DATA_TYPE agent_rectangle_min_side = std::min(agent_rectangle_length, agent_rectangle_width);
    FP_DATA_TYPE reward = reward_calculator.calculate_state_reward(&state);
    rectangle = new sf::RectangleShape(sf::Vector2f(agent_rectangle_length, agent_rectangle_width));
    sf::Vector2f agent_rectangle_position = agent_base_shape_position
            - to_sfml_vec(0.5f * trig_buff->get_rot_mat(-rotation)
                          * geometry::Vec(agent_rectangle_length, agent_rectangle_width));
    rectangle->setPosition(agent_rectangle_position);
    rectangle->setRotation(-180.0f * rotation / M_PI);
    */

    geometry::ORect agent_rect;
    agent->get_rect_variable()->get_value(agent_rect);

    sf::VertexArray *agent_shape = new sf::VertexArray(sf::PrimitiveType::TriangleStrip, 4);
    for (size_t i = 0; i < 4; ++i)
    {
        geometry::Vec rect_point = agent_rect[i];
        (*agent_shape)[i].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                               -get_pixels_per_metre() * rect_point.y());
        (*agent_shape)[i].color = sf::Color::Green;
    }
    //shape->setFillColor(to_sfml_colour(driving_agent_class_constant->get_value()));
    //shape->setOutlineThickness(agent_rectangle_min_side * 0.2f);
    //shape->setOutlineColor(sf::Color(
    //                           uint8_t(std::min(255.0f * 2.0f * (1.0f - reward), 255.0f)),
    //                           uint8_t(std::min(255.0f * 2.0f * reward, 255.0f)), 0));

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position += agent_rect.get_origin();
        focal_agent_count++;
    }

    /*
    FP_DATA_TYPE agent_circle_radius = 0.25f * agent_rectangle_min_side;
    circle = new sf::CircleShape(agent_circle_radius);
    sf::Vector2f agent_circle_position = agent_base_shape_position - sf::Vector2f(agent_circle_radius, agent_circle_radius);
    circle->setPosition(agent_circle_position);
    circle->setOutlineThickness(agent_circle_radius * 0.4f);
    if (focal_entities->contains(vehicle->get_name()))
    {
        circle->setOutlineColor(sf::Color(255, 255, 255));
    }
    else
    {
        circle->setOutlineColor(sf::Color(0, 0, 0));
    }

    FP_DATA_TYPE agent_text_size = 0.4f * agent_rectangle_min_side;
    text = new sf::Text(std::to_string(id_constant->get_value()), text_font, agent_text_size);
    sf::FloatRect text_bounds = text->getGlobalBounds();
    sf::Vector2f agent_text_position = agent_base_shape_position
            - 0.5f * sf::Vector2f(text_bounds.width, agent_text_size);
    text->setPosition(agent_text_position);
    text->setFillColor(sf::Color(0, 0, 0));
    */

    add_to_render_stack(agent_shape);
    //render_stack.push_back(circle);
    //render_stack.push_back(text);
}

void QAgentsWidget::on_init()
{
    causal::VariableContext::set_time_step_size(
                std::chrono::duration_cast<temporal::Duration>(frame_interval));

    /*
    // TODO: Make this non-OS specific
    if(!text_font.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf"))
    {
        text_enabled = false;
    }
    */

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
    start_time(start_time), end_time(end_time), current_time(start_time), update_required(false) {}

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

    if (last_realtime != temporal::Time::min())
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

    if (last_realtime != temporal::Time::min())
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


#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/agent_interface.hpp>
#include <ori/simcars/visualisation/utils.hpp>
#include <ori/simcars/visualisation/qscene_widget.hpp>


#include <iostream>

namespace ori
{
namespace simcars
{
namespace visualisation
{

QSceneWidget::QSceneWidget(std::shared_ptr<const agent::IScene> scene, QWidget* parent, const QPoint& position,
                           const QSize& size, FP_DATA_TYPE frame_rate, FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre)
    : AQSFMLCanvas(parent, position, size, int(1000.0f / frame_rate)), text_enabled(true), scene(scene), focused(false),
      focal_position(geometry::Vec::Zero()), focal_ego_agents(new structures::stl::STLStackArray<uint32_t>()),
      focal_non_ego_agents(new structures::stl::STLStackArray<uint32_t>()),
      realtime_factor(realtime_factor), pixels_per_metre(pixels_per_metre),
      earliest_time(scene->get_earliest_birth()), latest_time(scene->get_last_non_simulated_death()), current_time(earliest_time),
      last_time(temporal::Time::min()), last_realtime(temporal::Time::min()), update_required(true),
      trig_buff(geometry::TrigBuff::get_instance()) {}

void QSceneWidget::on_init()
{
    if(!text_font.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf"))
    {
        text_enabled = false;
    }

    on_update();
}

void QSceneWidget::on_update()
{
    if (mutex.try_lock())
    {
        const std::lock_guard<std::recursive_mutex> lock(mutex, std::adopt_lock);

        tick_forwards();

        if (update_required)
        {
            render_stack.clear();

            populate_render_stack();

            sf::View view(to_sfml_vec(focal_position), sf::Vector2f(this->width(), this->height()));
            setView(view);

            update_required = false;
        }

        clear(sf::Color(0, 0, 0));
        size_t i;
        for (i = 0; i < render_stack.count(); ++i)
        {
            draw(*(render_stack[i]));
        }
    }
}

void QSceneWidget::add_agent_to_render_stack(std::shared_ptr<const agent::IAgent> agent)
{
    FP_DATA_TYPE agent_rectangle_length = get_pixels_per_metre() * agent->get_length();
    FP_DATA_TYPE agent_rectangle_width = get_pixels_per_metre() * agent->get_width();
    FP_DATA_TYPE agent_rectangle_min_side = std::min(agent_rectangle_length, agent_rectangle_width);
    std::shared_ptr<sf::RectangleShape> rectangle(
                new sf::RectangleShape(sf::Vector2f(agent_rectangle_length, agent_rectangle_width)));
    std::shared_ptr<const agent::IAgent::State> agent_state = agent->get_state(get_time());
    sf::Vector2f agent_base_shape_position = to_sfml_vec(get_pixels_per_metre() * agent_state->position);
    sf::Vector2f agent_rectangle_position = agent_base_shape_position
            - to_sfml_vec(0.5f * trig_buff->get_rot_mat(agent_state->rotation)
                          * geometry::Vec(agent_rectangle_length, agent_rectangle_width));
    rectangle->setPosition(agent_rectangle_position);
    rectangle->setRotation(180 * agent_state->rotation / M_PI);
    rectangle->setFillColor(to_sfml_colour(agent->get_class()));
    rectangle->setOutlineThickness(agent_rectangle_min_side * 0.1f);
    rectangle->setOutlineColor(to_sfml_colour(agent_state->status));

    FP_DATA_TYPE agent_circle_radius = 0.25f * agent_rectangle_min_side;
    std::shared_ptr<sf::CircleShape> circle(new sf::CircleShape(agent_circle_radius));
    sf::Vector2f agent_circle_position = agent_base_shape_position - sf::Vector2f(agent_circle_radius, agent_circle_radius);
    circle->setPosition(agent_circle_position);
    circle->setOutlineThickness(agent_circle_radius * 0.4f);
    if (agent->is_ego())
    {
        circle->setOutlineColor(sf::Color(255, 255, 255));
    }
    else
    {
        circle->setOutlineColor(sf::Color(0, 0, 0));
    }
    if (agent->is_simulated(get_time()))
    {
        circle->setFillColor(sf::Color(0, 0, 255));
    }
    else
    {
        circle->setFillColor(circle->getOutlineColor());
    }

    FP_DATA_TYPE agent_text_size = 0.4f * agent_rectangle_min_side;
    std::shared_ptr<sf::Text> text(new sf::Text(std::to_string(agent->get_id()), text_font, agent_text_size));
    sf::FloatRect text_bounds = text->getGlobalBounds();
    sf::Vector2f agent_text_position = agent_base_shape_position
            - 0.5f * sf::Vector2f(text_bounds.width, agent_text_size);
    text->setPosition(agent_text_position);
    if (agent->is_simulated(get_time()))
    {
        text->setFillColor(sf::Color(255, 255, 0));
    }
    else
    {
        if (agent->is_ego())
        {
            text->setFillColor(sf::Color(0, 0, 0));
        }
        else
        {
            text->setFillColor(sf::Color(255, 255, 255));
        }
    }

    render_stack.push_back(rectangle);
    render_stack.push_back(circle);
    render_stack.push_back(text);
}

void QSceneWidget::add_scene_to_render_stack()
{
    scene->perform_simulations(get_time());

    std::shared_ptr<const structures::IArray<std::shared_ptr<const agent::IAgent>>> ego_agents =
            scene->get_ego_agents();
    std::shared_ptr<const structures::IArray<std::shared_ptr<const agent::IAgent>>> non_ego_agents =
            scene->get_non_ego_agents();

    focal_position = geometry::Vec::Zero();
    size_t focal_agent_count = 0;

    size_t i;
    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        try
        {
            std::shared_ptr<const agent::IAgent> agent = (*non_ego_agents)[i];

            add_agent_to_render_stack(agent);

            if (focused)
            {
                if (focal_non_ego_agents->contains(agent->get_id()))
                {
                    focal_position += get_pixels_per_metre() * agent->get_state(get_time())->position;
                    ++focal_agent_count;
                }
            }
            else
            {
                focal_position += get_pixels_per_metre() * agent->get_state(get_time())->position;
                ++focal_agent_count;
            }
        }
        catch (const std::out_of_range&) {}
    }

    for (i = 0; i < ego_agents->count(); ++i)
    {
        try
        {
            std::shared_ptr<const agent::IAgent> agent = (*ego_agents)[i];

            add_agent_to_render_stack(agent);

            if (focused)
            {
                if (focal_ego_agents->contains(agent->get_id()))
                {
                    focal_position += get_pixels_per_metre() * agent->get_state(get_time())->position;
                    ++focal_agent_count;
                }
            }
            else
            {
                focal_position += get_pixels_per_metre() * agent->get_state(get_time())->position;
                ++focal_agent_count;
            }
        }
        catch (const std::out_of_range&) {}
    }

    focal_position /= focal_agent_count;
}

void QSceneWidget::populate_render_stack()
{
    add_scene_to_render_stack();
}


FP_DATA_TYPE QSceneWidget::get_pixels_per_metre() const
{
    return pixels_per_metre;
}

const geometry::Vec& QSceneWidget::get_focal_position() const
{
    return focal_position;
}

std::shared_ptr<const structures::IArray<uint32_t>> QSceneWidget::get_focal_ego_agents() const
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    return focal_ego_agents;
}

std::shared_ptr<const structures::IArray<uint32_t>> QSceneWidget::get_focal_non_ego_agents() const
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    return focal_non_ego_agents;
}

temporal::Time QSceneWidget::get_time() const
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    return current_time;
}

void QSceneWidget::set_focal_ego_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_ego_agents)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    this->focal_ego_agents = focal_ego_agents;

    focused = focal_ego_agents->count() > 0 || focal_non_ego_agents->count() > 0;

    update_required = true;
}

void QSceneWidget::set_focal_non_ego_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_non_ego_agents)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    this->focal_non_ego_agents = focal_non_ego_agents;

    focused = focal_ego_agents->count() > 0 || focal_non_ego_agents->count() > 0;

    update_required = true;
}

void QSceneWidget::set_focal_agents(std::shared_ptr<const structures::IArray<uint32_t>> focal_ego_agents,
                                    std::shared_ptr<const structures::IArray<uint32_t>> focal_non_ego_agents)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    this->focal_ego_agents = focal_ego_agents;
    this->focal_non_ego_agents = focal_non_ego_agents;

    focused = focal_ego_agents->count() > 0 || focal_non_ego_agents->count() > 0;

    update_required = true;
}

void QSceneWidget::set_time(temporal::Time time)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    if (time < earliest_time)
    {
        current_time = earliest_time;
    }
    else if (time > latest_time)
    {
        current_time = latest_time;
    }
    else
    {
        current_time = time;
    }

    update_required = true;
}

void QSceneWidget::tick_forwards()
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    temporal::Time current_realtime = std::chrono::time_point_cast<temporal::Duration>(std::chrono::steady_clock::now());

    if (last_realtime != temporal::Time::min())
    {
        temporal::Duration realtime_diff = current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        last_time = current_time;
        if (current_time + time_diff > latest_time)
        {
            current_time = latest_time;
        }
        else
        {
            current_time += time_diff;
        }
    }

    last_realtime = current_realtime;

    if (last_time != current_time)
    {
        update_required = true;
    }
}

void QSceneWidget::tick_backwards()
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    temporal::Time current_realtime = std::chrono::time_point_cast<temporal::Duration>(std::chrono::steady_clock::now());

    last_time = current_time;
    if (last_realtime != temporal::Time::min())
    {
        temporal::Duration realtime_diff = current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        if (current_time - time_diff < earliest_time)
        {
            current_time = earliest_time;
        }
        else
        {
            current_time -= time_diff;
        }
    }

    last_realtime = current_realtime;

    if (last_time != current_time)
    {
        update_required = true;
    }
}

}
}
}

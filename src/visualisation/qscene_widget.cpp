
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/view_read_only_driving_agent_state.hpp>
#include <ori/simcars/visualisation/utils.hpp>
#include <ori/simcars/visualisation/qscene_widget.hpp>

#include <iostream>
#include <algorithm>

namespace ori
{
namespace simcars
{
namespace visualisation
{

QSceneWidget::QSceneWidget(agent::IScene const *scene, QWidget *parent, QPoint const &position,
                           QSize const &size, FP_DATA_TYPE frame_rate, FP_DATA_TYPE realtime_factor,
                           FP_DATA_TYPE pixels_per_metre, bool flip_y)
    : AQSFMLCanvas(parent, position, size, int(1000.0f / frame_rate)), text_enabled(true), scene(scene),
      realtime_factor(realtime_factor), pixels_per_metre(pixels_per_metre), flip_y(flip_y), focus_mode(FocusMode::FIXED),
      focal_position(geometry::Vec::Zero()), focal_entities(new structures::stl::STLStackArray<std::string>()),
      current_time(scene->get_min_temporal_limit()), last_time(temporal::Time::min()),
      last_realtime(temporal::Time::min()), update_required(true),
      trig_buff(geometry::TrigBuff::get_instance())
{
}

QSceneWidget::~QSceneWidget()
{
    delete focal_entities;

    for (size_t i = 0; i < render_stack.count(); ++i)
    {
        delete render_stack[i];
    }
}

void QSceneWidget::on_init()
{
    // TODO: Make this non-OS specific
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
        std::lock_guard<std::recursive_mutex> const lock(mutex, std::adopt_lock);

        tick_forwards();

        if (update_required)
        {
            for (size_t i = 0; i < render_stack.count(); ++i)
            {
                delete render_stack[i];
            }

            render_stack.clear();

            populate_render_stack();

            sf::View view(to_sfml_vec(get_pixels_per_metre() * focal_position, false, this->get_flip_y()),
                          sf::Vector2f(this->width(), this->height()));
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

void QSceneWidget::add_vehicle_to_render_stack(agent::IEntity const *vehicle)
{
    agent::IValuelessConstant const *ego_valueless_constant =
            vehicle->get_constant_parameter(vehicle->get_name() + ".ego");
    agent::IValuelessConstant const *id_valueless_constant =
            vehicle->get_constant_parameter(vehicle->get_name() + ".id");
    agent::IValuelessConstant const *driving_agent_class_valueless_constant =
            vehicle->get_constant_parameter(vehicle->get_name() + ".driving_agent_class");
    agent::IValuelessConstant const *bb_length_valueless_constant =
            vehicle->get_constant_parameter(vehicle->get_name() + ".bb_length");
    agent::IValuelessConstant const *bb_width_valueless_constant =
            vehicle->get_constant_parameter(vehicle->get_name() + ".bb_width");

    agent::IValuelessVariable const *position_valueless_variable =
            vehicle->get_variable_parameter(vehicle->get_name() + ".position.base");
    agent::IValuelessVariable const *rotation_valueless_variable =
            vehicle->get_variable_parameter(vehicle->get_name() + ".rotation.base");

    if (ego_valueless_constant == nullptr ||
            id_valueless_constant == nullptr ||
            driving_agent_class_valueless_constant == nullptr ||
            bb_length_valueless_constant == nullptr ||
            bb_width_valueless_constant == nullptr ||
            position_valueless_variable == nullptr ||
            rotation_valueless_variable == nullptr)
    {
        std::cerr << "Vehicle entity missing a required parameter" << std::endl;
        return;
    }

    agent::IConstant<bool> const *ego_constant =
            dynamic_cast<agent::IConstant<bool> const*>(ego_valueless_constant);
    agent::IConstant<uint32_t> const *id_constant =
            dynamic_cast<agent::IConstant<uint32_t> const*>(id_valueless_constant);
    agent::IConstant<agent::DrivingAgentClass> const *driving_agent_class_constant =
            dynamic_cast<agent::IConstant<agent::DrivingAgentClass> const*>(driving_agent_class_valueless_constant);
    agent::IConstant<FP_DATA_TYPE> const *bb_length_constant =
            dynamic_cast<agent::IConstant<FP_DATA_TYPE> const*>(bb_length_valueless_constant);
    agent::IConstant<FP_DATA_TYPE> const *bb_width_constant =
            dynamic_cast<agent::IConstant<FP_DATA_TYPE> const*>(bb_width_valueless_constant);

    agent::IVariable<geometry::Vec> const *position_variable =
            dynamic_cast<agent::IVariable<geometry::Vec> const*>(position_valueless_variable);
    agent::IVariable<FP_DATA_TYPE> const *rotation_variable =
            dynamic_cast<agent::IVariable<FP_DATA_TYPE> const*>(rotation_valueless_variable);

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
    FP_DATA_TYPE reward = reward_calculator.calculate_reward(&state);
    rectangle = new sf::RectangleShape(sf::Vector2f(agent_rectangle_length, agent_rectangle_width));
    sf::Vector2f agent_rectangle_position = agent_base_shape_position
            - to_sfml_vec(0.5f * trig_buff->get_rot_mat(-rotation)
                          * geometry::Vec(agent_rectangle_length, agent_rectangle_width));
    rectangle->setPosition(agent_rectangle_position);
    rectangle->setRotation(-180.0f * rotation / M_PI);
    rectangle->setFillColor(to_sfml_colour(driving_agent_class_constant->get_value()));
    rectangle->setOutlineThickness(agent_rectangle_min_side * 0.2f);
    rectangle->setOutlineColor(sf::Color(
                                   uint8_t(std::min(255.0f * 2.0f * (1.0f - reward), 255.0f)),
                                   uint8_t(std::min(255.0f * 2.0f * reward, 255.0f)),
                                   0));

    FP_DATA_TYPE agent_circle_radius = 0.25f * agent_rectangle_min_side;
    circle = new sf::CircleShape(agent_circle_radius);
    sf::Vector2f agent_circle_position = agent_base_shape_position - sf::Vector2f(agent_circle_radius, agent_circle_radius);
    circle->setPosition(agent_circle_position);
    circle->setOutlineThickness(agent_circle_radius * 0.4f);
    if (ego_constant->get_value())
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

    render_stack.push_back(rectangle);
    render_stack.push_back(circle);
    render_stack.push_back(text);
}

void QSceneWidget::add_scene_to_render_stack()
{
    structures::IArray<agent::IEntity const*> *entities = scene->get_entities();

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position = geometry::Vec::Zero();
    }
    size_t focal_agent_count = 0;

    size_t i;
    for (i = 0; i < entities->count(); ++i)
    {
        agent::IEntity const *entity = (*entities)[i];
        if (entity->get_name().find("vehicle") != std::string::npos)
        {
            add_vehicle_to_render_stack(entity);

            if (focus_mode == FocusMode::ALL_AGENTS ||
                    (focus_mode == FocusMode::FOCAL_AGENTS &&
                     focal_entities->contains(entity->get_name())))
            {
                agent::IValuelessVariable const *position_valueless_variable =
                        entity->get_variable_parameter(entity->get_name() + ".position.base");

                if (position_valueless_variable == nullptr)
                {
                    if (focus_mode == FocusMode::FOCAL_AGENTS)
                    {
                        std::cerr << "Focal entity '" << entity->get_name() << "' does not have a position parameter" << std::endl;
                    }
                    continue;
                }

                agent::IVariable<geometry::Vec> const *position_variable =
                        dynamic_cast<agent::IVariable<geometry::Vec> const*>(position_valueless_variable);

                geometry::Vec position;
                if (!position_variable->get_value(this->get_time(), position))
                {
                    continue;
                }
                focal_position += position;
                ++focal_agent_count;
            }
        }
    }

    delete entities;

    if (focus_mode == FocusMode::ALL_AGENTS || focus_mode == FocusMode::FOCAL_AGENTS)
    {
        focal_position /= focal_agent_count;
    }
}

void QSceneWidget::populate_render_stack()
{
    add_scene_to_render_stack();
}

FP_DATA_TYPE QSceneWidget::get_pixels_per_metre() const
{
    return pixels_per_metre;
}

bool QSceneWidget::get_flip_y() const
{
    return flip_y;
}

QSceneWidget::FocusMode QSceneWidget::get_focus_mode() const
{
    return focus_mode;
}

geometry::Vec const& QSceneWidget::get_focal_position() const
{
    return focal_position;
}

structures::IArray<std::string> const* QSceneWidget::get_focal_entities() const
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    return focal_entities;
}

temporal::Time QSceneWidget::get_time() const
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    return current_time;
}

void QSceneWidget::set_pixels_per_metre(FP_DATA_TYPE pixels_per_metre)
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    this->pixels_per_metre = pixels_per_metre;

    // TODO: Possibly add a check to see if an update is actually required?
    update_required = true;
}

void QSceneWidget::set_focus_mode(FocusMode focus_mode)
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    this->focus_mode = focus_mode;

    // TODO: Possibly add a check to see if an update is actually required?
    update_required = true;
}

void QSceneWidget::set_focal_position(geometry::Vec const &focal_position)
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    this->focal_position = focal_position;

    // TODO: Possibly add a check to see if an update is actually required?
    if (focus_mode == FocusMode::FIXED)
    {
        update_required = true;
    }

}

void QSceneWidget::set_focal_entities(structures::IArray<std::string> const *focal_entities)
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

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

void QSceneWidget::set_time(temporal::Time time)
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    if (time < scene->get_min_temporal_limit())
    {
        current_time = scene->get_min_temporal_limit();
    }
    else if (time > scene->get_max_temporal_limit())
    {
        current_time = scene->get_max_temporal_limit();
    }
    else
    {
        current_time = time;
    }

    update_required = true;
}

void QSceneWidget::tick_forwards()
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    temporal::Time current_realtime = std::chrono::time_point_cast<temporal::Duration>(std::chrono::steady_clock::now());

    if (last_realtime != temporal::Time::min())
    {
        temporal::Duration realtime_diff = current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        if (current_time + time_diff > scene->get_max_temporal_limit())
        {
            current_time = scene->get_max_temporal_limit();
        }
        else
        {
            current_time += time_diff;
        }
    }

    last_realtime = current_realtime;

    if (last_time != current_time)
    {
        last_time = current_time;
        update_required = true;
    }
}

void QSceneWidget::tick_backwards()
{
    std::lock_guard<std::recursive_mutex> const lock(mutex);

    temporal::Time current_realtime = std::chrono::time_point_cast<temporal::Duration>(std::chrono::steady_clock::now());

    if (last_realtime != temporal::Time::min())
    {
        temporal::Duration realtime_diff = current_realtime - last_realtime;
        temporal::Duration time_diff = std::chrono::duration_cast<temporal::Duration>(realtime_factor * realtime_diff);

        if (current_time - time_diff < scene->get_min_temporal_limit())
        {
            current_time = scene->get_min_temporal_limit();
        }
        else
        {
            current_time -= time_diff;
        }
    }

    last_realtime = current_realtime;

    if (last_time != current_time)
    {
        last_time = current_time;
        update_required = true;
    }
}

}
}
}

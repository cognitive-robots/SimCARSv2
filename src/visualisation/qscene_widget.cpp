
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
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
      focal_position(geometry::Vec::Zero()), focal_entities(new structures::stl::STLStackArray<std::string>()),
      realtime_factor(realtime_factor), pixels_per_metre(pixels_per_metre), current_time(scene->get_min_temporal_limit()),
      last_time(temporal::Time::min()), last_realtime(temporal::Time::min()), update_required(true),
      trig_buff(geometry::TrigBuff::get_instance()) {}

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
        const std::lock_guard<std::recursive_mutex> lock(mutex, std::adopt_lock);

        tick_forwards();

        if (update_required)
        {
            render_stack.clear();

            populate_render_stack();

            sf::View view(to_sfml_vec(focal_position, false, true), sf::Vector2f(this->width(), this->height()));
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

void QSceneWidget::add_vehicle_to_render_stack(std::shared_ptr<const agent::IEntity> vehicle)
{
    try
    {
        std::shared_ptr<const agent::IValuelessConstant> ego_valueless_constant =
                vehicle->get_constant_parameter(vehicle->get_name() + ".ego");
        std::shared_ptr<const agent::IValuelessConstant> id_valueless_constant =
                vehicle->get_constant_parameter(vehicle->get_name() + ".id");
        std::shared_ptr<const agent::IValuelessConstant> road_agent_class_valueless_constant =
                vehicle->get_constant_parameter(vehicle->get_name() + ".road_agent_class");
        std::shared_ptr<const agent::IValuelessConstant> bb_length_valueless_constant =
                vehicle->get_constant_parameter(vehicle->get_name() + ".bb_length");
        std::shared_ptr<const agent::IValuelessConstant> bb_width_valueless_constant =
                vehicle->get_constant_parameter(vehicle->get_name() + ".bb_width");

        std::shared_ptr<const agent::IConstant<bool>> ego_constant =
                std::static_pointer_cast<const agent::IConstant<bool>>(ego_valueless_constant);
        std::shared_ptr<const agent::IConstant<uint32_t>> id_constant =
                std::static_pointer_cast<const agent::IConstant<uint32_t>>(id_valueless_constant);
        std::shared_ptr<const agent::IConstant<agent::DrivingAgentClass>> road_agent_class_constant =
                std::static_pointer_cast<const agent::IConstant<agent::DrivingAgentClass>>(road_agent_class_valueless_constant);
        std::shared_ptr<const agent::IConstant<FP_DATA_TYPE>> bb_length_constant =
                std::static_pointer_cast<const agent::IConstant<FP_DATA_TYPE>>(bb_length_valueless_constant);
        std::shared_ptr<const agent::IConstant<FP_DATA_TYPE>> bb_width_constant =
                std::static_pointer_cast<const agent::IConstant<FP_DATA_TYPE>>(bb_width_valueless_constant);

        FP_DATA_TYPE agent_rectangle_length = get_pixels_per_metre() * bb_length_constant->get_value();
        FP_DATA_TYPE agent_rectangle_width = get_pixels_per_metre() * bb_width_constant->get_value();
        FP_DATA_TYPE agent_rectangle_min_side = std::min(agent_rectangle_length, agent_rectangle_width);
        std::shared_ptr<sf::RectangleShape> rectangle(
                    new sf::RectangleShape(sf::Vector2f(agent_rectangle_length, agent_rectangle_width)));

        std::shared_ptr<const agent::IValuelessVariable> position_valueless_variable =
                vehicle->get_variable_parameter(vehicle->get_name() + ".position.base");
        std::shared_ptr<const agent::IValuelessVariable> rotation_valueless_variable =
                vehicle->get_variable_parameter(vehicle->get_name() + ".rotation.base");

        std::shared_ptr<const agent::IVariable<geometry::Vec>> position_variable =
                std::static_pointer_cast<const agent::IVariable<geometry::Vec>>(position_valueless_variable);
        std::shared_ptr<const agent::IVariable<FP_DATA_TYPE>> rotation_variable =
                std::static_pointer_cast<const agent::IVariable<FP_DATA_TYPE>>(rotation_valueless_variable);

        temporal::Time current_time = this->get_time();
        try
        {
            sf::Vector2f agent_base_shape_position = to_sfml_vec(get_pixels_per_metre() * position_variable->get_value(current_time), false, true);
            sf::Vector2f agent_rectangle_position = agent_base_shape_position
                    - to_sfml_vec(0.5f * trig_buff->get_rot_mat(-rotation_variable->get_value(current_time))
                                  * geometry::Vec(agent_rectangle_length, agent_rectangle_width));
            rectangle->setPosition(agent_rectangle_position);
            rectangle->setRotation(-180 * rotation_variable->get_value(current_time) / M_PI);
            rectangle->setFillColor(to_sfml_colour(road_agent_class_constant->get_value()));
            rectangle->setOutlineThickness(agent_rectangle_min_side * 0.1f);

            FP_DATA_TYPE agent_circle_radius = 0.25f * agent_rectangle_min_side;
            std::shared_ptr<sf::CircleShape> circle(new sf::CircleShape(agent_circle_radius));
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
            std::shared_ptr<sf::Text> text(new sf::Text(std::to_string(id_constant->get_value()), text_font, agent_text_size));
            sf::FloatRect text_bounds = text->getGlobalBounds();
            sf::Vector2f agent_text_position = agent_base_shape_position
                    - 0.5f * sf::Vector2f(text_bounds.width, agent_text_size);
            text->setPosition(agent_text_position);
            text->setFillColor(sf::Color(0, 0, 0));

            render_stack.push_back(rectangle);
            render_stack.push_back(circle);
            render_stack.push_back(text);
        }
        catch (std::out_of_range)
        {
            //std::cerr << "Position / Rotation not available for this time" << std::endl;
        }
    }
    catch (std::out_of_range)
    {
        std::cerr << "Vehicle entity missing a required parameter" << std::endl;
    }
}

void QSceneWidget::add_scene_to_render_stack()
{
    std::shared_ptr<structures::IArray<std::shared_ptr<const agent::IEntity>>> entities = scene->get_entities();

    focal_position = geometry::Vec::Zero();
    size_t focal_agent_count = 0;

    size_t i;
    for (i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<const agent::IEntity> entity = (*entities)[i];
        if (entity->get_name().find("vehicle") != std::string::npos)
        {
            add_vehicle_to_render_stack(entity);

            if (!focused || focal_entities->contains(entity->get_name()))
            {
                try
                {
                    std::shared_ptr<const agent::IValuelessVariable> position_valueless_variable =
                            entity->get_variable_parameter(entity->get_name() + ".position.base");

                    std::shared_ptr<const agent::IVariable<geometry::Vec>> position_variable =
                            std::static_pointer_cast<const agent::IVariable<geometry::Vec>>(position_valueless_variable);

                    // TODO: Replace this with something more efficient
                    try
                    {
                        geometry::Vec position = position_variable->get_value(this->get_time());
                        focal_position += get_pixels_per_metre() * position;
                        ++focal_agent_count;
                    }
                    catch (std::out_of_range)
                    {
                        //std::cerr << "Position not available for this time" << std::endl;
                    }
                }
                catch (std::out_of_range)
                {
                    if (focused)
                    {
                        std::cerr << "Focal entity '" << entity->get_name() << "' does not have a position parameter" << std::endl;
                    }
                }
            }
        }
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

std::shared_ptr<const structures::IArray<std::string>> QSceneWidget::get_focal_entities() const
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    return focal_entities;
}

temporal::Time QSceneWidget::get_time() const
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    return current_time;
}

void QSceneWidget::set_focal_entities(std::shared_ptr<const structures::IArray<std::string>> focal_entities)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

    this->focal_entities = focal_entities;

    focused = focal_entities->count() > 0;

    // TODO: Possibly add a check to see if an update is actually required?
    update_required = true;
}

void QSceneWidget::set_time(temporal::Time time)
{
    const std::lock_guard<std::recursive_mutex> lock(mutex);

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
    const std::lock_guard<std::recursive_mutex> lock(mutex);

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
    const std::lock_guard<std::recursive_mutex> lock(mutex);

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

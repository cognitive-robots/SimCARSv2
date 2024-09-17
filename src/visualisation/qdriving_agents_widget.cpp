
#include <ori/simcars/visualisation/qdriving_agents_widget.hpp>

#include <ori/simcars/geometry/o_rect.hpp>
#include <ori/simcars/causal/variable_context.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace visualisation
{

void QDrivingAgentsWidget::add_agent_to_render_stack(agents::FWDCar *agent)
{
    geometry::ORect agent_rect;
    bool res = agent->get_rect_variable()->get_value(agent_rect);

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

        FP_DATA_TYPE motor_torque;
        agent->get_motor_torque_variable()->get_value(motor_torque);
        FP_DATA_TYPE steer;
        agent->get_steer_variable()->get_value(steer);
        geometry::Vec lin_vel;
        agent->get_lin_vel_variable()->get_value(lin_vel);
        FP_DATA_TYPE lon_lin_vel;
        agent->get_lon_lin_vel_variable()->get_value(lon_lin_vel);
        geometry::Vec lin_acc;
        agent->get_lin_acc_variable()->get_value(lin_acc);
        geometry::Vec env_force;
        agent->get_env_force_variable()->get_value(env_force);
        geometry::Vec other_force;
        agent->get_other_force_variable()->get_value(other_force);
        FP_DATA_TYPE dist_headway;
        agent->get_dist_headway_variable()->get_value(dist_headway);
        std::cout << "Motor Torque: " << motor_torque << " Nm, " << "Steer: " << steer << " rad, " <<
                     "Lin. Vel.: (" << lin_vel.x() << ", " << lin_vel.y() << ") m/s, " <<
                     "Lon. Lin. Vel.: " << lon_lin_vel << " m/s, " <<
                     "Lin. Acc.: (" << lin_acc.x() << ", " << lin_acc.y() << ") m/s^2, " <<
                     "Env. Force: (" << env_force.x() << ", " << env_force.y() << ") N, " <<
                     "Other Force: (" << other_force.x() << ", " << other_force.y() << ") N, " <<
                     "Dist. Headway: " << dist_headway << " m" <<
                     std::endl;
        /*
        geometry::Vec other_force;
        agent->get_other_force_variable()->get_value(other_force);
        geometry::Vec env_force;
        agent->get_env_force_variable()->get_value(env_force);
        FP_DATA_TYPE mass;
        agent->get_mass_variable()->get_value(mass);
        geometry::Vec lin_acc;
        agent->get_lin_acc_variable()->get_value(lin_acc);
        geometry::Vec lin_vel;
        agent->get_lin_vel_variable()->get_value(lin_vel);
        geometry::Vec dir;
        agent->get_dir_variable()->get_value(dir);
        FP_DATA_TYPE lon_lin_vel;
        agent->get_lon_lin_vel_variable()->get_value(lon_lin_vel);
        FP_DATA_TYPE lat_lin_vel;
        agent->get_lat_lin_vel_variable()->get_value(lat_lin_vel);
        FP_DATA_TYPE front_wheel_ang_lat_lin_vel;
        agent->get_front_wheel_ang_lat_lin_vel_variable()->get_value(front_wheel_ang_lat_lin_vel);
        FP_DATA_TYPE front_wheel_slip_ang;
        agent->get_front_wheel_slip_ang()->get_value(front_wheel_slip_ang);
        FP_DATA_TYPE actual_front_wheel_slip_ang;
        agent->get_actual_front_wheel_slip_ang()->get_value(actual_front_wheel_slip_ang);
        FP_DATA_TYPE front_wheel_lat_force_mag;
        agent->get_front_wheel_lat_force_mag_variable()->get_value(front_wheel_lat_force_mag);
        FP_DATA_TYPE rear_wheel_ang_lat_lin_vel;
        agent->get_rear_wheel_ang_lat_lin_vel_variable()->get_value(rear_wheel_ang_lat_lin_vel);
        FP_DATA_TYPE rear_wheel_slip_ang;
        agent->get_rear_wheel_slip_ang()->get_value(rear_wheel_slip_ang);
        FP_DATA_TYPE actual_rear_wheel_slip_ang;
        agent->get_actual_rear_wheel_slip_ang()->get_value(actual_rear_wheel_slip_ang);
        FP_DATA_TYPE rear_wheel_lat_force_mag;
        agent->get_rear_wheel_lat_force_mag_variable()->get_value(rear_wheel_lat_force_mag);
        FP_DATA_TYPE other_torque;
        agent->get_other_torque_variable()->get_value(other_torque);
        FP_DATA_TYPE ang_acc;
        agent->get_ang_acc_variable()->get_value(ang_acc);
        FP_DATA_TYPE ang_vel;
        agent->get_ang_vel_variable()->get_value(ang_vel);
        std::cout << "Other Force: (" << other_force.x() << ", " << other_force.y() << ") N, " <<
                     "Env. Force: (" << env_force.x() << ", " << env_force.y() << ") N, " <<
                     "Mass: " << mass << " kg, " <<
                     "Lin. Acc.: (" << lin_acc.x() << ", " << lin_acc.y() << ") m/s^2, " <<
                     "Lin. Vel.: (" << lin_vel.x() << ", " << lin_vel.y() << ") m/s, " <<
                     "Dir.: (" << dir.x() << ", " << dir.y() << "), " <<
                     "Lat. Lin. Vel.: " << lat_lin_vel << " m/s, " <<
                     "Lon. Lin. Vel.: " << lon_lin_vel << " m/s, " <<
                     "Front Wheel Ang. Lat. Lin. Vel.: " << front_wheel_ang_lat_lin_vel << " m/s, " <<
                     "Front Wheel Slip Ang.: " << front_wheel_slip_ang << " rad, " <<
                     "Actual Front Wheel Slip Ang.: " << actual_front_wheel_slip_ang << " rad, " <<
                     "Front Wheel Lat. Force Mag.: " << front_wheel_lat_force_mag << " N, " <<
                     "Rear Wheel Ang. Lat. Lin. Vel.: " << rear_wheel_ang_lat_lin_vel << " m/s, " <<
                     "Rear Wheel Slip Ang.: " << rear_wheel_slip_ang << " rad, " <<
                     "Actual Rear Wheel Slip Ang.: " << actual_rear_wheel_slip_ang << " rad, " <<
                     "Rear Wheel Lat. Force Mag.: " << rear_wheel_lat_force_mag << " N, " <<
                     "Other Torque: " << other_torque << " Nm, " <<
                     "Ang. Acc: " << ang_acc << " rad/s^2, " <<
                     "Ang. Vel: " << ang_vel << " rad/s" <<
                     std::endl;
                     */
    }
    else
    {
        agent_colour = sf::Color::Green;
    }

    sf::VertexArray *agent_shape = new sf::VertexArray(sf::PrimitiveType::TriangleStrip, 4);

    geometry::Vec rect_point = agent_rect[0];
    (*agent_shape)[0].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[0].color = agent_colour;

    rect_point = agent_rect[3];
    (*agent_shape)[1].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[1].color = agent_colour;

    rect_point = agent_rect[1];
    (*agent_shape)[2].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[2].color = agent_colour;

    rect_point = agent_rect[2];
    (*agent_shape)[3].position = sf::Vector2f(get_pixels_per_metre() * rect_point.x(),
                                           -get_pixels_per_metre() * rect_point.y());
    (*agent_shape)[3].color = agent_colour;

    if (focus_mode == FocusMode::ALL_AGENTS ||
            (focus_mode == FocusMode::FOCAL_AGENTS && res && focal_agent_ids->contains(id)))
    {
        focal_position += agent_rect.get_origin();
        focal_agent_count++;
    }

    add_to_render_stack(agent_shape);

    if (text_enabled)
    {
        sf::Text *agent_text = new sf::Text(std::to_string(id), text_font, 16);
        agent_text->setPosition(sf::Vector2f(get_pixels_per_metre() * agent_rect.get_origin().x(),
                                             -get_pixels_per_metre() * agent_rect.get_origin().y()));
        add_to_render_stack(agent_text);
    }
}

void QDrivingAgentsWidget::on_init()
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

void QDrivingAgentsWidget::on_update()
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

void QDrivingAgentsWidget::add_to_render_stack(sf::Drawable const *drawable)
{
    render_stack.push_back(drawable);
}

void QDrivingAgentsWidget::add_agents_to_render_stack()
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

void QDrivingAgentsWidget::populate_render_stack()
{
    add_agents_to_render_stack();
}

QDrivingAgentsWidget::QDrivingAgentsWidget(
        QWidget *parent, QPoint const &position, QSize const &size, temporal::Time start_time,
        temporal::Time end_time, std::chrono::milliseconds frame_interval,
        FP_DATA_TYPE realtime_factor, FP_DATA_TYPE pixels_per_metre, FocusMode focus_mode) :
    AQSFMLCanvas(parent, position, size, frame_interval), text_enabled(true),
    frame_interval(frame_interval), realtime_factor(realtime_factor),
    pixels_per_metre(pixels_per_metre), focus_mode(focus_mode), focal_agent_ids(nullptr),
    start_time(start_time), end_time(end_time), current_time(start_time),
    last_realtime(std::chrono::time_point<std::chrono::steady_clock>::min()),
    update_required(false) {}

QDrivingAgentsWidget::~QDrivingAgentsWidget()
{
    for (size_t i = 0; i < render_stack.count(); ++i)
    {
        delete render_stack[i];
    }
}

bool QDrivingAgentsWidget::get_text_enabled() const
{
    return text_enabled;
}

FP_DATA_TYPE QDrivingAgentsWidget::get_realtime_factor() const
{
    return realtime_factor;
}

FP_DATA_TYPE QDrivingAgentsWidget::get_pixels_per_metre() const
{
    return pixels_per_metre;
}

QDrivingAgentsWidget::FocusMode QDrivingAgentsWidget::get_focus_mode() const
{
    return focus_mode;
}

geometry::Vec const& QDrivingAgentsWidget::get_focal_position() const
{
    return focal_position;
}

structures::IArray<uint64_t> const* QDrivingAgentsWidget::get_focal_agent_ids() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return focal_agent_ids;
}

temporal::Time QDrivingAgentsWidget::get_time() const
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    return current_time;
}

void QDrivingAgentsWidget::set_text_enabled(bool text_enabled)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    this->text_enabled = text_enabled;
}

void QDrivingAgentsWidget::set_realtime_factor(FP_DATA_TYPE realtime_factor)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    this->realtime_factor = realtime_factor;
}

void QDrivingAgentsWidget::set_pixels_per_metre(FP_DATA_TYPE pixels_per_metre)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->pixels_per_metre != pixels_per_metre)
    {
        this->pixels_per_metre = pixels_per_metre;
        update_required = true;
    }
}

void QDrivingAgentsWidget::set_focus_mode(FocusMode focus_mode)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (this->focus_mode != focus_mode)
    {
        this->focus_mode = focus_mode;
        update_required = true;
    }
}

void QDrivingAgentsWidget::set_focal_position(geometry::Vec const &focal_position)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (focus_mode != FocusMode::FIXED || this->focal_position != focal_position)
    {
        focus_mode = FocusMode::FIXED;
        this->focal_position = focal_position;
        update_required = true;
    }
}

void QDrivingAgentsWidget::set_focal_agent_ids(structures::IArray<uint64_t> const *focal_agent_ids)
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

void QDrivingAgentsWidget::set_agent_colour(uint64_t agent_id, sf::Color colour)
{
    std::lock_guard<std::recursive_mutex> const lock(data_mutex);

    if (id_colour_dict.contains(agent_id) && id_colour_dict[agent_id] != colour)
    {
        update_required = true;
    }

    id_colour_dict.update(agent_id, colour);
}

void QDrivingAgentsWidget::set_time(temporal::Time time)
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

void QDrivingAgentsWidget::tick_forwards()
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

void QDrivingAgentsWidget::tick_backwards()
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

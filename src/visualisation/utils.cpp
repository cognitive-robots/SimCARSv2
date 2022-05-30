
#include <ori/simcars/visualisation/utils.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

sf::Vector2f to_sfml_vec(const geometry::Vec& vec)
{
    return sf::Vector2f(vec.x(), vec.y());
}

sf::Color to_sfml_colour(map::ITrafficLightStateHolder::FaceColour face_colour)
{
    switch (face_colour)
    {
    case map::ITrafficLightStateHolder::FaceColour::UNKNOWN:
        break;

    case map::ITrafficLightStateHolder::FaceColour::RED:
        return sf::Color(255, 0, 0);

    case map::ITrafficLightStateHolder::FaceColour::YELLOW:
        return sf::Color(255, 255, 0);

    case map::ITrafficLightStateHolder::FaceColour::GREEN:
        return sf::Color(0, 255, 0);
    }

    return sf::Color(255, 255, 255);
}

sf::Color to_sfml_colour(agent::IAgent::Class agent_class)
{
    switch(agent_class)
    {
    case agent::IAgent::Class::UNKNOWN:
        break;

    case agent::IAgent::Class::CAR:
        return sf::Color(255, 128, 0);

    case agent::IAgent::Class::VAN:
        return sf::Color(255, 255, 0);

    case agent::IAgent::Class::TRAM:
        return sf::Color(128, 255, 0);

    case agent::IAgent::Class::BUS:
        return sf::Color(0, 255, 0);

    case agent::IAgent::Class::TRUCK:
        return sf::Color(0, 255, 128);

    case agent::IAgent::Class::EMERGENCY_VEHICLE:
        return sf::Color(0, 255, 255);

    case agent::IAgent::Class::BICYCLE:
        return sf::Color(0, 128, 255);

    case agent::IAgent::Class::MOTORCYCLE:
        return sf::Color(0, 0, 255);

    case agent::IAgent::Class::PEDESTRIAN:
        return sf::Color(128, 0, 255);

    case agent::IAgent::Class::ANIMAL:
        return sf::Color(255, 0, 255);

    case agent::IAgent::Class::OTHER_VEHICLE:
        return sf::Color(255, 0, 128);
    }

    return sf::Color(255, 255, 255);
}

sf::Color to_sfml_colour(agent::IAgent::Status agent_status)
{
    switch(agent_status)
    {
    case agent::IAgent::Status::UNKNOWN:
        break;

    case agent::IAgent::Status::NOMINAL:
        return sf::Color(128, 255, 0);

    case agent::IAgent::Status::WARNING:
        return sf::Color(255, 255, 0);

    case agent::IAgent::Status::ERROR:
        return sf::Color(255, 128, 0);

    case agent::IAgent::Status::FATAL:
        return sf::Color(255, 0, 0);
    }

    return sf::Color(255, 255, 255);
}

}
}
}

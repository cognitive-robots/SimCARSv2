
#include <ori/simcars/visualisation/utils.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

sf::Vector2f to_sfml_vec(const geometry::Vec& vec, bool flip_x, bool flip_y)
{
    FP_DATA_TYPE x;
    FP_DATA_TYPE y;
    if (flip_x)
    {
        x = -vec.x();
    }
    else
    {
        x = vec.x();
    }
    if (flip_y)
    {
        y = -vec.y();
    }
    else
    {
        y = vec.y();
    }
    return sf::Vector2f(x, y);
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

sf::Color to_sfml_colour(agent::lyft::RoadAgentClass road_agent_class)
{
    switch(road_agent_class)
    {
    case agent::lyft::RoadAgentClass::UNKNOWN:
        break;

    case agent::lyft::RoadAgentClass::CAR:
        return sf::Color(255, 128, 0);

    case agent::lyft::RoadAgentClass::VAN:
        return sf::Color(255, 255, 0);

    case agent::lyft::RoadAgentClass::TRAM:
        return sf::Color(128, 255, 0);

    case agent::lyft::RoadAgentClass::BUS:
        return sf::Color(0, 255, 0);

    case agent::lyft::RoadAgentClass::TRUCK:
        return sf::Color(0, 255, 128);

    case agent::lyft::RoadAgentClass::EMERGENCY_VEHICLE:
        return sf::Color(0, 255, 255);

    case agent::lyft::RoadAgentClass::BICYCLE:
        return sf::Color(0, 128, 255);

    case agent::lyft::RoadAgentClass::MOTORCYCLE:
        return sf::Color(0, 0, 255);

    case agent::lyft::RoadAgentClass::PEDESTRIAN:
        return sf::Color(128, 0, 255);

    case agent::lyft::RoadAgentClass::ANIMAL:
        return sf::Color(255, 0, 255);

    case agent::lyft::RoadAgentClass::OTHER_VEHICLE:
        return sf::Color(255, 0, 128);
    }

    return sf::Color(255, 255, 255);
}

/*
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
*/

}
}
}

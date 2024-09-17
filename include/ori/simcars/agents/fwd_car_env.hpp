#pragma once

#include <ori/simcars/agents/rect_rigid_body_env.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCarEnv : public RectRigidBodyEnv
{
    class Entity
    {
        class Link
        {
            uint64_t id;
            FWDCar *fwd_car;
            uint64_t other_id;
            FWDCar *other_fwd_car;

        protected:
            simcars::causal::ORectDistHeadwayVariable dist_headway;

        public:
            Link(uint64_t id, FWDCar *fwd_car, uint64_t other_id, WDCar *other_fwd_car);

            simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_dist_headway();
        };

        uint64_t id;
        FWDCar *fwd_car;
        structures::stl::STLDictionary<uint64_t, Link*> id_link_dict;

    protected:
        simcars::causal::ScalarFixedVariable dist_headway_limit;
        simcars::causal::ScalarProxyVariable dist_headway_limit_proxy;

        simcars::causal::ScalarSetMinVariable min_dist_headway;

    public:
        Entity(uint64_t id, FWDCar *fwd_car);

        virtual ~Entity();

        simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_dist_headway();

        bool add_link(FWDCar *other_fwd_car);
        bool remove_link(FWDCar *other_fwd_car);
    };

    structures::stl::STLDictionary<uint64_t, FWDCar*> id_fwd_car_dict;
    structures::stl::STLDictionary<uint64_t, Entity*> id_entity_dict;

public:
    virtual ~FWDCarEnv();

    structures::IArray<FWDCar*> const* get_fwd_cars() const;

    bool add_fwd_car(FWDCar *fwd_car);
    bool remove_fwd_car(FWDCar *fwd_car);
};

}
}
}

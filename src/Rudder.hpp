#ifndef GAZEBO_THRUSTER_RUDDER_HPP
#define GAZEBO_THRUSTER_RUDDER_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

namespace gazebo_thruster {
    class Actuators;
    class Rudder {
    public:
        Rudder(Actuators& actuators, gazebo::physics::ModelPtr model,
               sdf::ElementPtr sdf);
        ~Rudder();

        void update(Actuators& actuator);

    private:
        std::string mLinkName;
        gazebo::physics::LinkPtr mLink;
        size_t mActuatorID;

        float mFluidDensity = 1000;
        float mArea = 1;
        float mLiftK = 1.5;
        float mDragK = 1;
    };
}

#endif

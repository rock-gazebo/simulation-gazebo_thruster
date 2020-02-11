#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

#include "msgs.pb.h"

namespace gazebo_thruster {
    class Actuators;

    /** Management of all the thrusters in a given model */
    class Thrusters {
    public:
        void load(
            Actuators& actuators, gazebo::transport::NodePtr node,
            gazebo::physics::ModelPtr model, sdf::ElementPtr pluginElement
        );
        void update(Actuators& actuators);

    private:
        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters>
            ThrustersMSG;

        void processThrusterCommand(ThrustersMSG const& thrustersMSG);

        struct Definition {
            std::string name;
            size_t actuatorID;
            gazebo::physics::LinkPtr link;
            double minThrust;
            double maxThrust;
            double effort;
        };

        std::vector<Definition> mDefinitions;

        gazebo::physics::ModelPtr mModel;
        gazebo::transport::SubscriberPtr mCommandSubscriber;

        std::vector<Definition> loadThrusters(
            Actuators& actuators, sdf::ElementPtr pluginElement
        );

        /** Apply the min/max thrust to thruster effort
         */
        void clampThrustEffort(Definition& thruster);
    };
}

#endif

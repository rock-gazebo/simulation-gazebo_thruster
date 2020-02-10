#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>
#include "Actuator.hpp"

#include "msgs.pb.h"

namespace gazebo_thruster
{
    class GazeboThruster : public gazebo::ModelPlugin
    {
    public:
        GazeboThruster();
        ~GazeboThruster();
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters>
            ThrustersMSG;

        void readInput(ThrustersMSG const& thrustersMSG);

        struct Thruster{
            std::string name;
            gazebo::physics::LinkPtr link;
            double minThrust;
            double maxThrust;
            double effort;
            ignition::math::Vector3d added_mass_compensated_direction;
            ignition::math::Vector3d added_mass_compensated_position;
        };

        gazebo_underwater::Matrix6 mass_matrix;

    private:
        Actuator mActuator;

        void updateBegin(gazebo::common::UpdateInfo const& info);
        std::vector<Thruster> loadThrusters();
        void initComNode();

        /** Apply the min/max thrust to thruster effort
         */
        void clampThrustEffort(Thruster& thruster);

        gazebo::event::ConnectionPtr mWorldUpdateEvent;
        gazebo::transport::NodePtr mNode;
        gazebo::physics::ModelPtr mModel;
        gazebo::transport::SubscriberPtr mThrusterSubscriber;

        std::vector<Thruster> thrusters;
    };
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
}

#endif

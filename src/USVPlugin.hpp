#ifndef GAZEBO_THRUSTER_PLUGIN_HPP
#define GAZEBO_THRUSTER_PLUGIN_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "Actuators.hpp"
#include "Rudder.hpp"
#include "Thrusters.hpp"

namespace gazebo_thruster {
    class USVPlugin : public gazebo::ModelPlugin {
    public:
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        gazebo::event::ConnectionPtr mWorldUpdateEvent;
        gazebo::transport::NodePtr mNode;
        gazebo::physics::ModelPtr mModel;

        Actuators mActuators;
        std::vector<Rudder> mRudders;
        Thrusters mThrusters;

        void updateBegin(gazebo::common::UpdateInfo const& info);

        void loadRudders(sdf::ElementPtr pluginElement);
        void loadThrusters(sdf::ElementPtr pluginElement);
    };

    GZ_REGISTER_MODEL_PLUGIN(USVPlugin);
}

#endif
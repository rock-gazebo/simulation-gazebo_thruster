#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "msgs.pb.h"

namespace gazebo_thruster
{
    class GazeboThruster : public gazebo::ModelPlugin
    {
    public:
        GazeboThruster();
        ~GazeboThruster();
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters> ThrustersMSG;
        void readInput(ThrustersMSG &thrustersMSG);

        struct Thruster{
            std::string name;
            double minThrust;
            double maxThrust;
            double effort;
        };

    private:
        void updateBegin(gazebo::common::UpdateInfo const& info);
        double thrusterMathModel(double input);
        void loadThrusters();
        void initComNode();
        void checkThrusters();
        void checkThrustLimits(std::vector<Thruster>::iterator thruster);
        double updateEffort(gazebo_thruster::msgs::Thruster thrusterCMD);

        std::vector<gazebo::event::ConnectionPtr> eventHandler;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr thrusterSubscriber;
        gazebo::physics::ModelPtr model;

        template <typename T>
        T getParameter(sdf::ElementPtr thrusterElement, std::string parameter_name,
                std::string dimension, T default_value);
        std::vector<Thruster> thrusters;
    };
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
} 

#endif

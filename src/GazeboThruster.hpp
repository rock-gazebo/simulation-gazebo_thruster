#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "thruster_types.pb.h"

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

    private:
        void updateBegin(gazebo::common::UpdateInfo const& info);
        double thrusterMathModel(double input);
        void loadLinks();
        void initComNode();

        std::vector<gazebo::event::ConnectionPtr> eventHandler;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr thrusterSubscriber;
        gazebo::physics::ModelPtr model;

        typedef std::map<std::string,double> ThrusterOutput;
        ThrusterOutput thrusterOutput;
    };
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
} 

#endif

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

            typedef const boost::shared_ptr<const rock_thruster::msgs::Joints> JointsMSG;
            void readInput(JointsMSG &jointsMSG);

        private:
            void updateBegin(gazebo::common::UpdateInfo const& info);
            double thrusterMathModel(double input);
            void loadLinks();
            void initNode();

            std::vector<gazebo::event::ConnectionPtr> eventHandler;
            gazebo::transport::NodePtr node;
            gazebo::transport::SubscriberPtr thrusterSubscriber;

            typedef std::map<std::string,double> GazeboJoint;
            GazeboJoint thrusterInput;
            gazebo::physics::ModelPtr model;
	};
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
} 

#endif

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
        private:
            void updateBegin(gazebo::common::UpdateInfo const& info);
            double thrusterMathModel();
            void applyForce(double force);

            double thruster_input;
            gazebo::physics::LinkPtr link;
            std::vector<gazebo::event::ConnectionPtr> eventHandler;
            gazebo::transport::NodePtr node;

		public:
            GazeboThruster();
            ~GazeboThruster();
			virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

            typedef const boost::shared_ptr<const rock_thruster::msgs::ThrusterInput> ThrusterInput;
            void readInput(ThrusterInput &_thruster_input);
	};
    GZ_REGISTER_MODEL_PLUGIN(GazeboThruster)
} 

#endif

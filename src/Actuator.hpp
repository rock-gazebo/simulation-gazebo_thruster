#ifndef GAZEBO_THRUSTER_ACTUATOR_HPP
#define GAZEBO_THRUSTER_ACTUATOR_HPP

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

namespace gazebo_thruster {
    /**
     * Class that applies added mass corrections on forces before applying them
     * to the underlying model
     */
    class Actuator {
        typedef const boost::shared_ptr<const gazebo_underwater::msgs::CompensatedMass> \
            CompensatedMassMSG;

        gazebo::transport::SubscriberPtr mCompensatedMassSubscriber;

        void readCompensatedMass(CompensatedMassMSG const& msg);
        void updateCompensatedEffort(
            gazebo_underwater::Matrix6 const& matrix,
            ignition::math::Vector3d const& cog
        );
        gazebo_underwater::Matrix6 mCurrentMassMatrix;
        ignition::math::Vector3d mCurrentCOG;

        struct AddedMassCompensation {
            gazebo::physics::LinkPtr link;
            ignition::math::Vector3d direction;
            ignition::math::Vector3d compensated_direction;
            ignition::math::Vector3d compensated_position;
        };

        std::map<gazebo::physics::LinkPtr, AddedMassCompensation>
            mAddedMassCompensation;

    public:
        Actuator();
        Actuator(gazebo::physics::ModelPtr model, gazebo::transport::NodePtr node);

        void addLink(gazebo::physics::LinkPtr link, ignition::math::Vector3d direction);
        void applyForce(gazebo::physics::LinkPtr link, float magnitude);
    };
}

#endif

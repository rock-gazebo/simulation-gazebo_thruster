#include "Actuator.hpp"
#include <gazebo/physics/physics.hh>

using namespace gazebo_thruster;
using namespace std;
using namespace ignition::math;

Actuator::Actuator() {
}

Actuator::Actuator(gazebo::physics::ModelPtr model, gazebo::transport::NodePtr node) {
    string topicName = model->GetName() + "/compensated_mass";
    mCompensatedMassSubscriber = node->Subscribe(
        "~/" + topicName, &Actuator::readCompensatedMass, this, true
    );
}

void Actuator::readCompensatedMass(CompensatedMassMSG const& msg) {
    gazebo_underwater::Matrix6 matrix(
        msg->matrix() - gazebo_underwater::Matrix6::Identity()
    );

    if (mCurrentMassMatrix != matrix)
    {
        Vector3d cog(gazebo::msgs::ConvertIgn(msg->cog()));
        updateCompensatedEffort(matrix, cog);
        mCurrentCOG = cog;
        mCurrentMassMatrix = matrix;
    }
}

void Actuator::addLink(gazebo::physics::LinkPtr link,
                       ignition::math::Vector3d direction) {
    AddedMassCompensation info;
    info.link = link;
    info.direction = direction;
    mAddedMassCompensation[link] = info;
    updateCompensatedEffort(mCurrentMassMatrix, mCurrentCOG);
}

void Actuator::applyForce(gazebo::physics::LinkPtr link, float magnitude) {
    auto it = mAddedMassCompensation.find(link);
    if (it == mAddedMassCompensation.end()) {
        throw std::invalid_argument("link not registered, use addLink first");
    }

    auto const& info = it->second;
    link->AddLinkForce(magnitude * info.direction);
    link->AddLinkForce(magnitude * info.compensated_direction,
                       info.compensated_position);
}

void Actuator::updateCompensatedEffort(
    gazebo_underwater::Matrix6 const& matrix,
    Vector3d const& cog
) {
    for (auto& entry : mAddedMassCompensation) {
        gazebo::physics::LinkPtr link = entry.first;
        auto& linkInfo = entry.second;
        auto linkRelativePose = GzGetIgn((*link), RelativePose, ());

        Vector3d force(linkRelativePose.Rot().RotateVector(linkInfo.direction));
        gazebo_underwater::Vector6 effort(
            force,
            (linkRelativePose.Pos() - cog).Cross(force)
        );
        effort = matrix * effort;

        auto compensated_direction =
            linkRelativePose.Rot().RotateVectorReverse(effort.top);
        auto compensated_position =
            effort.top.Cross(effort.bottom) / compensated_direction.SquaredLength()
            + cog - linkRelativePose.Pos();

        linkInfo.compensated_direction = compensated_direction;
        linkInfo.compensated_position = compensated_position;
    }
}


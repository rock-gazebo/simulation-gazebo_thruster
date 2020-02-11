#include "Actuators.hpp"
#include <gazebo/physics/physics.hh>

using namespace gazebo_thruster;
using namespace std;
using namespace ignition::math;

Actuators::Actuators() {
}

Actuators::Actuators(gazebo::physics::ModelPtr model, gazebo::transport::NodePtr node) {
    string topicName = model->GetName() + "/compensated_mass";
    mCompensatedMassSubscriber = node->Subscribe(
        "~/" + topicName, &Actuators::readCompensatedMass, this, true
    );
}

void Actuators::readCompensatedMass(CompensatedMassMSG const& msg) {
    gazebo_underwater::Matrix6 matrix(
        msg->matrix() - gazebo_underwater::Matrix6::Identity()
    );

    if (mCurrentMassMatrix != matrix) {
        for (auto& info : mAddedMassCompensation) {
            info.lastModelRelativeDirection = Vector3d(0, 0, 0);
        }
        mCurrentCOG = gazebo::msgs::ConvertIgn(msg->cog());
        mCurrentMassMatrix = matrix;
    }
}

size_t Actuators::addLink(gazebo::physics::LinkPtr link) {
    AddedMassCompensation info;
    info.link = link;
    int id = mAddedMassCompensation.size();
    mAddedMassCompensation.push_back(info);
    return id;
}

void Actuators::applyForce(size_t id, Vector3d const& force) {
    if (mAddedMassCompensation.size() < id) {
        throw std::invalid_argument("invalid ID in applyForce");
    }

    auto& info = mAddedMassCompensation[id];
    auto linkRelativePose = GzGetIgn((*info.link), RelativePose, ());
    auto direction = force.Normalized();
    auto modelRelativeDirection = linkRelativePose.Rot().RotateVector(direction);

    if (info.lastModelRelativeDirection != modelRelativeDirection) {
        updateCompensatedEffort(info, direction, mCurrentMassMatrix, mCurrentCOG);
        info.lastModelRelativeDirection = modelRelativeDirection;
    }
    info.link->AddLinkForce(force);
    info.link->AddLinkForce(force.Length() * info.compensatedDirection,
                            info.compensatedPosition);
}

void Actuators::updateCompensatedEffort(
    AddedMassCompensation& info,
    Vector3d const& modelRelativeDirection,
    gazebo_underwater::Matrix6 const& matrix,
    Vector3d const& cog
) {
    auto linkRelativePose = GzGetIgn((*info.link), RelativePose, ());

    gazebo_underwater::Vector6 effort(
        modelRelativeDirection,
        (linkRelativePose.Pos() - cog).Cross(modelRelativeDirection)
    );
    effort = matrix * effort;

    auto compensated_direction =
        linkRelativePose.Rot().RotateVectorReverse(effort.top);
    auto compensated_position =
        effort.top.Cross(effort.bottom) / compensated_direction.SquaredLength()
        + cog - linkRelativePose.Pos();

    info.compensatedDirection = compensated_direction;
    info.compensatedPosition = compensated_position;
}

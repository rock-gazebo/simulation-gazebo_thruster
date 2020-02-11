#include <gazebo_thruster/USVPlugin.hpp>
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_thruster;

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    mModel = _model;
    // Import all thrusters from a model file (sdf)
    sdf::ElementPtr modelSDF = mModel->GetSDF();
    sdf::ElementPtr pluginElement = utilities::getPluginElement(
        modelSDF, "libgazebo_thruster.so"
    );

    // Initialize communication node and subscribe to gazebo topic
    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();

    mActuators = Actuators(mModel, mNode);

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    loadRudders(pluginElement);
    loadThrusters(pluginElement);
}

void USVPlugin::loadRudders(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("rudder");
    while (el) {
        mRudders.push_back(Rudder(mActuators, mModel, el));
        el = el->GetNextElement("rudder");
    }
}

void USVPlugin::loadThrusters(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("thruster");
    if (el) {
        mThrusters.load(mActuators, mNode, mModel, pluginElement);
    }
}

void USVPlugin::updateBegin(common::UpdateInfo const& info) {
    for (auto& rudder : mRudders) {
        rudder.update(mActuators);
    }

    mThrusters.update(mActuators);
}

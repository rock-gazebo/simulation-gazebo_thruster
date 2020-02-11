#include "Thrusters.hpp"
#include "Utilities.hpp"
#include "Actuators.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_thruster;
using namespace ignition::math;

void Thrusters::load(
    Actuators& actuators, transport::NodePtr node,
    physics::ModelPtr model, sdf::ElementPtr pluginElement
) {
    mModel = model;
    mDefinitions = loadThrusters(actuators, pluginElement);

    // Initialize communication node and subscribe to gazebo topic
    string topicName = mModel->GetName() + "/thrusters";
    mCommandSubscriber =
        node->Subscribe("~/" + topicName, &Thrusters::processThrusterCommand, this);

    auto worldName = GzGet((*mModel->GetWorld()), Name, ());
    gzmsg << "Thruster: receiving thruster commands from /gazebo/"
          << worldName << "/" << topicName << endl;
}

std::vector<Thrusters::Definition> Thrusters::loadThrusters(
    Actuators& actuators, sdf::ElementPtr pluginElement
) {
    std::vector<Definition> definitions;
    sdf::ElementPtr el = pluginElement->GetElement("thruster");
    while (el) {
        // Load thrusters attributes
        Definition def;
        def.name = el->Get<string>("name");
        auto link = mModel->GetLink(def.name);
        if (!link) {
            gzthrow("Thruster: thruster " + def.name + " does not exist");
        }

        gzmsg << "Thruster: thruster name: " << def.name << endl;
        def.actuatorID = actuators.addLink(link);
        def.link = link;
        def.minThrust = utilities::getParameter<double>(
            "Thruster", el, "min_thrust", "N", -200
        );
        def.maxThrust = utilities::getParameter<double>(
            "Thruster", el, "max_thrust", "N", 200
        );
        def.effort = 0.0;
        definitions.push_back(def);
        el = el->GetNextElement("thruster");
    }

    if (definitions.empty())
    {
        string msg = "Thruster: sdf model loads thruster plugin but has no\n"
                     "thruster defined. Please name the links you want to export\n"
                     "as thrusters inside the <plugin> tag, e.g.:\n"
                     "<thruster name='thruster::right'> ";
        gzthrow(msg);
    }
    return definitions;
}

void Thrusters::processThrusterCommand(ThrustersMSG const& thrustersMSG) {
    for (int i = 0; i < thrustersMSG->thrusters_size(); ++i) {
        bool thrusterFound = false;
        const gazebo_thruster::msgs::Thruster& thrusterCMD = thrustersMSG->thrusters(i);
        for (auto& thruster : mDefinitions) {
            if (thrusterCMD.name() == thruster.name) {
                thrusterFound = true;
                thruster.effort = thrusterCMD.has_effort() ? thrusterCMD.effort() : 0;
                clampThrustEffort(thruster);
            }
        }
        if (!thrusterFound) {
            gzthrow("Thruster: incoming thruster name: "
                    + thrusterCMD.name() + ", not found.");
        }
    }
}

void Thrusters::clampThrustEffort(Definition& thruster)
{
    if (thruster.effort < thruster.minThrust) {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " below the minimum\n"
              << "Thruster: using minThrust: " << thruster.minThrust
              << " instead" << endl;
        thruster.effort = thruster.minThrust;
    }
    else if (thruster.effort > thruster.maxThrust)
    {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " above the maximum\n"
              << "Thruster: using maxThrust: " << thruster.maxThrust
              << " instead" << endl;
        thruster.effort = thruster.maxThrust;
    }
}

void Thrusters::update(Actuators& actuators) {
    for (auto& thruster : mDefinitions) {
        actuators.applyForce(thruster.actuatorID,
                             Vector3d::UnitX * thruster.effort);
    }
}

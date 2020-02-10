#include "GazeboThruster.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_thruster;
using namespace ignition::math;

GazeboThruster::GazeboThruster()
{
}


GazeboThruster::~GazeboThruster()
{
    mNode->Fini();
}

void GazeboThruster::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    mModel = _model;
    gzmsg << "GazeboThruster: loading thrusters from model: "
          << mModel->GetName() << endl;

    this->thrusters = loadThrusters();

    // Initialize communication node and subscribe to gazebo topic
    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();
    string topicName = mModel->GetName() + "/thrusters";
    mThrusterSubscriber =
        mNode->Subscribe("~/" + topicName, &GazeboThruster::readInput, this);

    mActuator = Actuator(mModel, mNode);

    auto worldName = GzGet((*mModel->GetWorld()), Name, ());
    gzmsg << "GazeboThruster: create gazebo topic /gazebo/"
          << worldName << "/" << topicName << endl;

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboThruster::updateBegin, this, _1)
    );
}

std::vector<gazebo_thruster::GazeboThruster::Thruster> GazeboThruster::loadThrusters()
{
    // Import all thrusters from a model file (sdf)
    sdf::ElementPtr modelSDF = mModel->GetSDF();

    sdf::ElementPtr pluginElement = utilities::getPluginElement(
        modelSDF, "libgazebo_thruster.so"
    );

    std::vector<Thruster> thrusters;
    sdf::ElementPtr thrusterElement = pluginElement->GetElement("thruster");
    while (thrusterElement)
    {
        // Load thrusters attributes
        Thruster thruster;
        thruster.name = thrusterElement->Get<string>("name");
        auto link = mModel->GetLink(thruster.name);
        if (!link) {
            gzthrow("Thruster: thruster " + thruster.name + " does not exist");
        }

        gzmsg << "GazeboThruster: thruster name: " << thruster.name << endl;
        mActuator.addLink(link, Vector3d::UnitX);
        thruster.link = link;
        thruster.minThrust = utilities::getParameter<double>(
            "GazeboThruster", thrusterElement, "min_thrust", "N", -200
        );
        thruster.maxThrust = utilities::getParameter<double>(
            "GazeboThruster", thrusterElement, "max_thrust", "N", 200
        );
        thruster.effort = 0.0;
        thrusters.push_back(thruster);
        thrusterElement = thrusterElement->GetNextElement("thruster");
    }

    if (thrusters.empty())
    {
        string msg = "GazeboThruster: sdf model loads thruster plugin but has no\n"
                     "thruster defined. Please name the links you want to export\n"
                     "as thrusters inside the <plugin> tag, e.g.:\n"
                     "<thruster name='thruster::right'> ";
        gzthrow(msg);
    }
    return thrusters;
}

void GazeboThruster::readInput(ThrustersMSG const& thrustersMSG) {
    for (int i = 0; i < thrustersMSG->thrusters_size(); ++i) {
        bool thrusterFound = false;
        const gazebo_thruster::msgs::Thruster& thrusterCMD = thrustersMSG->thrusters(i);
        for (auto& thruster : thrusters) {
            if (thrusterCMD.name() == thruster.name) {
                thrusterFound = true;
                thruster.effort = thrusterCMD.has_effort() ? thrusterCMD.effort() : 0;
                clampThrustEffort(thruster);
            }
        }
        if (!thrusterFound) {
            gzthrow("GazeboThruster: incoming thruster name: "
                    + thrusterCMD.name() + ", not found.");
        }
    }
}

void GazeboThruster::clampThrustEffort(Thruster& thruster)
{
    if (thruster.effort < thruster.minThrust) {
        gzmsg << "GazeboThruster: thruster effort " << thruster.effort
              << " below the minimum\n"
              << "GazeboThruster: using minThrust: " << thruster.minThrust
              << " instead" << endl;
        thruster.effort = thruster.minThrust;
    }
    else if (thruster.effort > thruster.maxThrust)
    {
        gzmsg << "GazeboThruster: thruster effort " << thruster.effort
              << " above the maximum\n"
              << "GazeboThruster: using maxThrust: " << thruster.maxThrust
              << " instead" << endl;
        thruster.effort = thruster.maxThrust;
    }
}

void GazeboThruster::updateBegin(common::UpdateInfo const& info) {
    for (auto& thruster : thrusters) {
        mActuator.applyForce(thruster.link, thruster.effort);
    }
}

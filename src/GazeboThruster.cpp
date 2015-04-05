#include "GazeboThruster.hpp"

using namespace gazebo;
using namespace gazebo_thruster;


GazeboThruster::GazeboThruster(): thruster_input(0) 
{
}

GazeboThruster::~GazeboThruster()
{
    node->Fini();
}


void GazeboThruster::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
    gzmsg << "GazeboThruster: Loading thruster." << std::endl;

    gazebo::physics::ModelPtr model = _model;
    link = model->GetLink("link_name");

    eventHandler.push_back(
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboThruster::updateBegin,this, _1)));

    // Initialize communication node
    node = transport::NodePtr(new transport::Node());
    node->Init();
    transport::SubscriberPtr sub = node->Subscribe(model->GetName(),
            &GazeboThruster::readInput,this);
}

void GazeboThruster::updateBegin(common::UpdateInfo const& info)
{
    applyForce( thrusterMathModel() );
}

void GazeboThruster::readInput(ThrusterInput &_thruster_input)
{
    thruster_input = _thruster_input->frequency();
}


double GazeboThruster::thrusterMathModel()
{
    // Linear model
    double a = 1.0, b = 0.0;
    double thruster_output = a* thruster_input + b;
    return thruster_output;
}

void GazeboThruster::applyForce(double force)
{
    link->AddForce( math::Vector3(force,0,0) );
}




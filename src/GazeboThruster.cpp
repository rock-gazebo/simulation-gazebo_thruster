#include "GazeboThruster.hpp"

using namespace gazebo;
using namespace gazebo_thruster;

GazeboThruster::GazeboThruster()
{
}

GazeboThruster::~GazeboThruster()
{
    node->Fini();
}

void GazeboThruster::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
    model = _model;
    gzmsg << "GazeboThruster: loading thrusters from model: " << model->GetName() << std::endl;

    loadLinks();
    initNode();

    eventHandler.push_back(
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboThruster::updateBegin,this, _1)));
}


void GazeboThruster::loadLinks()
{
    // Get all links and look for "underwater_thruster_body" in each one of them
    physics::Link_V links = model->GetLinks();
    for(physics::Link_V::iterator link = links.begin(); link != links.end(); ++link)
    {
        std::string linkName = (*link)->GetName();
        std::size_t found = linkName.find("underwater_thruster_body");
        if(found != std::string::npos )
        {
            gzmsg <<"GazeboThruster: found link: " << linkName << std::endl;
            linkName.erase( found - 2 );
            gzmsg <<"GazeboThruster: JointState element name in ROCK joint input port must match the link name: "
                    << linkName << std::endl;
            thrusterInput.insert( std::make_pair( linkName, 0.0 ) );
        }
    }
    if(thrusterInput.empty())
        gzthrow("GazeboThruster: thruster link name underwater_thruster_body not found.");
}


void GazeboThruster::initNode()
{
    // Initialize communication node and subscribe to gazebo topic
    node = transport::NodePtr(new transport::Node());
    node->Init();
    std::string topicName = model->GetName() + "/thruster";
    thrusterSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readInput,this);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ model->GetWorld()->GetName()
            + "/" + topicName << std::endl;
}


void GazeboThruster::readInput(JointsMSG& jointsMSG)
{
    // Read buffer and update joints data
    for(int i = 0; i < jointsMSG->thruster_size(); ++i)
    {
        const rock_thruster::msgs::Thruster& thruster = jointsMSG->thruster(i);
        GazeboJoint::iterator joint = thrusterInput.find( thruster.name() );
        if( joint != thrusterInput.end() )
        {
            if( thruster.has_raw() )
                joint->second = thrusterMathModel( thruster.raw() );

            if( thruster.has_effort() )
                joint->second = thruster.effort();
        }else{
            gzmsg << "GazeboThruster: thruster "<< thruster.name() << " not found." << std::endl;
        }
    }
}


double GazeboThruster::thrusterMathModel(double input)
{
    // Linear model
    double a = 1.0, b = 0.0;
    return a*( input ) + b;
}


void GazeboThruster::updateBegin(common::UpdateInfo const& info)
{
    for(GazeboJoint::iterator input = thrusterInput.begin();
            input != thrusterInput.end(); ++input)
    {
        physics::LinkPtr link = model->GetLink(input->first + "::underwater_thruster_body");
        link->AddForce( math::Vector3(input->second,0,0) );
    }
}

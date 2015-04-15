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
    initComNode();

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
            thrusterOutput.insert( std::make_pair( linkName, 0.0 ) );
        }
    }
    if(thrusterOutput.empty())
        gzthrow("GazeboThruster: thruster link name underwater_thruster_body not found.");
}


void GazeboThruster::initComNode()
{
    // Initialize communication node and subscribe to gazebo topic
    node = transport::NodePtr(new transport::Node());
    node->Init();
    std::string topicName = model->GetName() + "/thrusters";
    thrusterSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readInput,this);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ model->GetWorld()->GetName()
            + "/" + topicName << std::endl;
}


void GazeboThruster::readInput(ThrustersMSG& thrustersMSG)
{
    // Read buffer and update output data
    for(int i = 0; i < thrustersMSG->thrusters_size(); ++i)
    {
        const rock_thruster::msgs::Thruster& thruster = thrustersMSG->thrusters(i);
        ThrusterOutput::iterator output = thrusterOutput.find( thruster.name() );
        if( output != thrusterOutput.end() )
        {
            if( thruster.has_raw() )
                output->second = thrusterMathModel( thruster.raw() );

            if( thruster.has_effort() )
                output->second = thruster.effort();
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
    for(ThrusterOutput::iterator output = thrusterOutput.begin();
            output != thrusterOutput.end(); ++output)
    {
        physics::LinkPtr link = model->GetLink(output->first + "::underwater_thruster_body");
        link->AddForce( math::Vector3(output->second,0,0) );
    }
}

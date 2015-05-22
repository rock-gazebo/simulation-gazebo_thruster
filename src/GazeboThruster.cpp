#include "GazeboThruster.hpp"

using namespace std;
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
    gzmsg << "GazeboThruster: loading thrusters from model: " << model->GetName() << endl;

    std::vector<Thruster> thrusters = loadThrusters();
    if( checkThrusters(thrusters));
        this->thrusters = thrusters;

    initComNode();

    eventHandler.push_back(
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboThruster::updateBegin,this, _1)));
}


template <class T>
T GazeboThruster::getParameter(sdf::ElementPtr thrusterElement,
        string parameter_name, string dimension, T default_value)
{
    T var = default_value;
    if(thrusterElement->HasElement(parameter_name.c_str()))
    {
        var = thrusterElement->Get< T >(parameter_name.c_str());
        gzmsg << "GazeboThruster: " + parameter_name + ": " << var << " " +
                dimension  << endl;
    }else{
        gzmsg << "GazeboThruster: " + parameter_name + ": using default: "
                << default_value << " " + dimension << endl;
    }
    return var;
}


std::vector<gazebo_thruster::GazeboThruster::Thruster> GazeboThruster::loadThrusters()
{
    // Import all thrusters from a model file (sdf)
    std::vector<Thruster> thrusters;
    sdf::ElementPtr modelSDF = model->GetSDF();
    if (modelSDF->HasElement("plugin"))
    {
        sdf::ElementPtr pluginElement = modelSDF->GetElement("plugin");
        gzmsg << "GazeboThruster: found plugin (filename): " << pluginElement->Get<string>("filename") << endl;
        gzmsg << "GazeboThruster: found plugin (name): " << pluginElement->Get<string>("name") << endl;
        if(pluginElement->Get<string>("filename") == "libgazebo_thruster.so")
        {
            if(pluginElement->HasElement("thruster"))
            {
                sdf::ElementPtr thrusterElement = pluginElement->GetElement("thruster");
                while(thrusterElement)
                {
                    // Check thrusters attributes
                    Thruster thruster;
                    thruster.name = thrusterElement->Get<string>("name");
                    gzmsg << "GazeboThruster: thruster name: " << thruster.name << endl;
                    thruster.minThrust = getParameter<double>(thrusterElement,"min_thrust","N",-100);
                    thruster.maxThrust = getParameter<double>(thrusterElement,"max_thrust","N",100);
                    thruster.effort = 0.0;
                    thrusters.push_back(thruster);
                    thrusterElement = thrusterElement->GetNextElement("thruster");
                }
            }else
            {
                string msg = "GazeboThruster: sdf model loads thruster plugin but has no thruster defined.\n";
                msg += "GazeboThruster: please name the links you want to export as thrusters inside the <plugin> tag: \n";
                msg += "GazeboThruster: <thruster name='thruster::right'> ";
                gzthrow(msg);
            }
        }
    }
    return thrusters;
}


bool GazeboThruster::checkThrusters(std::vector<Thruster> thrusters)
{
    // Look for link names that match thrusters names
    for(vector<Thruster>::iterator thruster = thrusters.begin(); thruster != thrusters.end(); ++thruster)
    {
        if( !model->GetLink(thruster->name) )
        {
            string msg = "GazeboThruster: no link name match the thruster name: " + thruster->name +
                    " in gazebo model: " + model->GetName();
            gzthrow(msg);
        }
    }
    return true;
}


void GazeboThruster::initComNode()
{
    // Initialize communication node and subscribe to gazebo topic
    node = transport::NodePtr(new transport::Node());
    node->Init();
    string topicName = model->GetName() + "/thrusters";
    thrusterSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readInput,this);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ model->GetWorld()->GetName()
            + "/" + topicName << endl;
}


void GazeboThruster::readInput(ThrustersMSG& thrustersMSG)
{
    // Read buffer and update the thruster effort
    for(int i = 0; i < thrustersMSG->thrusters_size(); ++i)
    {
        bool thrusterFound = false;
        const gazebo_thruster::msgs::Thruster& thrusterCMD = thrustersMSG->thrusters(i);
        for(vector<Thruster>::iterator thruster = thrusters.begin();
                thruster != thrusters.end(); ++thruster)
        {
            if(thrusterCMD.name() == thruster->name)
            {
                thrusterFound = true;
                thruster->effort = updateEffort(thrusterCMD);
                checkThrustLimits(thruster);
            }
        }
        if(!thrusterFound)
            gzmsg << "GazeboThruster: incoming thruster name: "<< thrusterCMD.name() << ", not found." << endl;
    }
}


double GazeboThruster::updateEffort(gazebo_thruster::msgs::Thruster thrusterCMD)
{
    double effort;

    if( thrusterCMD.has_raw() )
        effort = thrusterMathModel( thrusterCMD.raw() );

    if( thrusterCMD.has_effort() )
        effort = thrusterCMD.effort();

    return effort;
}


void GazeboThruster::checkThrustLimits(vector<Thruster>::iterator thruster)
{
    if(thruster->effort < thruster->minThrust)
    {
        gzmsg << "GazeboThruster: thruster effort below the minimum: " << thruster->minThrust << endl;
        gzmsg << "GazeboThruster: using minThrust: " << thruster->minThrust << ", instead. " << endl;
        thruster->effort = thruster->minThrust;
    }

    if(thruster->effort > thruster->maxThrust)
    {
        gzmsg << "GazeboThruster: incoming thruster effort above the maximum: " << thruster->maxThrust << endl;
        gzmsg << "GazeboThruster: using maxThrust: " << thruster->maxThrust << ", instead. " << endl;
        thruster->effort = thruster->maxThrust;
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
    for(vector<Thruster>::iterator thruster = thrusters.begin();
            thruster != thrusters.end(); ++thruster)
    {
        physics::LinkPtr link = model->GetLink( thruster->name );
        link->AddRelativeForce( math::Vector3(thruster->effort,0,0) );
    }
}


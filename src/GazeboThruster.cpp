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
    compensated_effort = false;
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
                    // Load thrusters attributes
                    Thruster thruster;
                    thruster.name = thrusterElement->Get<string>("name");
                    gzmsg << "GazeboThruster: thruster name: " << thruster.name << endl;
                    thruster.minThrust = getParameter<double>(thrusterElement,"min_thrust","N",-200);
                    thruster.maxThrust = getParameter<double>(thrusterElement,"max_thrust","N",200);
                    thruster.effort = 0.0;
                    thrusters.push_back(thruster);
                    thruster.direction = gazebo::math::Vector3::UnitX;
                    thruster.position = gazebo::math::Vector3::Zero;
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

    topicName = model->GetName() + "/compensated_mass";
    compensatedMassSubscriber = node->Subscribe("~/" + topicName,&GazeboThruster::readCompensatedMass,this,true);
    gzmsg <<"GazeboThruster: create gazebo topic /gazebo/"+ model->GetWorld()->GetName()
            + "/" + topicName << endl;
}


void GazeboThruster::readInput(ThrustersMSG const& thrustersMSG)
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

void GazeboThruster::readCompensatedMass(CompMassMSG const& compMassMSG)
{
    gazebo_underwater::Matrix6 matrix(compMassMSG->matrix());
    gazebo::math::Vector3 cog(gazebo::msgs::ConvertIgn(compMassMSG->cog()));
    if(!compensated_effort)
    {
        updateCompensatedEffort(matrix, cog);
        compensated_effort = true;
    }
}

double GazeboThruster::updateEffort(gazebo_thruster::msgs::Thruster thrusterCMD)
{
    if( thrusterCMD.has_effort() ){
        return thrusterCMD.effort();
    }else{
        return 0;
    }
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


void GazeboThruster::updateBegin(common::UpdateInfo const& info)
{
    for(vector<Thruster>::iterator thruster = thrusters.begin();
            thruster != thrusters.end(); ++thruster)
    {
        physics::LinkPtr link = model->GetLink( thruster->name );
        link->AddLinkForce( thruster->effort*thruster->direction, thruster->position );
    }
}

void GazeboThruster::updateCompensatedEffort(gazebo_underwater::Matrix6 const& matrix, gazebo::math::Vector3 const& cog)
{
    gazebo_underwater::Vector6 effort;
    for(vector<Thruster>::iterator thruster = thrusters.begin();
            thruster != thrusters.end(); ++thruster)
    {
        physics::LinkPtr link = model->GetLink( thruster->name );
        effort.top = link->GetRelativePose().rot.RotateVector(gazebo::math::Vector3::UnitX);
        effort.bottom = (link->GetRelativePose().pos-cog).Cross(effort.top);
        gzmsg << "GazeboThruster: name: " << thruster->name << ", force: "<< effort.top << ", torque: "<< effort.bottom << endl;
        effort = matrix * effort;
        thruster->direction = link->GetRelativePose().rot.RotateVectorReverse(effort.top);
        thruster->position = effort.top.Cross(effort.bottom)/(thruster->direction.GetSquaredLength()) + cog - link->GetRelativePose().pos;
        thruster->position = link->GetRelativePose().rot.RotateVectorReverse(thruster->position);
        gzmsg << "GazeboThruster: name: " << thruster->name << ", direction: "<< thruster->direction << ", position: "<< thruster->position << endl;
    }
}

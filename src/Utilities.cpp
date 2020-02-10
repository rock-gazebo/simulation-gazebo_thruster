#include "Utilities.hpp"
#include <gazebo/common/Exception.hh>

using namespace gazebo_thruster;

sdf::ElementPtr utilities::findPluginElement(
    sdf::ElementPtr enclosing, std::string const& fileName
) {
    sdf::ElementPtr pluginElement = enclosing->GetElement("plugin");
    while (pluginElement)
    {
        if (pluginElement->Get<std::string>("filename") == fileName) {
            gzmsg << "Found plugin: " << pluginElement->Get<std::string>("name")
                  << " (" << fileName << ")" << std::endl;
            return pluginElement;
        }
        pluginElement = pluginElement->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr utilities::getPluginElement(
    sdf::ElementPtr enclosing, std::string const& fileName
) {
    auto element = findPluginElement(enclosing, fileName);

    if (!element) {
        std::string msg =
            "GazeboThruster: sdf model loaded the thruster plugin, but it\n"
            "cannot be found in the SDF object. Expected the thruster plugin\n"
            "filename to be libgazebo_thruster.so\n";
        gzthrow(msg);
    }

    return element;
}

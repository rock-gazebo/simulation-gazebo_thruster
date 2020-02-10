#ifndef GAZEBO_THRUSTER_UTILITIES_HPP
#define GAZEBO_THRUSTER_UTILITIES_HPP

#include <sdf/sdf.hh>
#include <gazebo/common/Console.hh>

namespace gazebo_thruster {
    namespace utilities {
        /** Looks for a <plugin> element
         *
         * This methods searches for a plugin element, direct child of
         * \c enclosing, whose filename is the given file name. It differs
         * from \c getPluginElement by the way it handles the missing case.
         *
         * @return a null pointer if the plugin could not be found
         * @see getPluginElement
         */
        sdf::ElementPtr findPluginElement(sdf::ElementPtr enclosing,
                                          std::string const& fileName);

        /** Resolves for a <plugin> element
         *
         * This methods searches for a plugin element, direct child of
         * \c enclosing, whose filename is the given file name. It differs
         * from \c findPluginElement by the way it handles the missing case.
         *
         * @throw invalid_argument if the plugin could not be found
         * @see findPluginElement
         */
        sdf::ElementPtr getPluginElement(sdf::ElementPtr enclosing,
                                         std::string const& fileName);


        /** Get a typed parameter from the given element
         *
         * If the parameter does not exist, use a default value
         */
        template <class T>
        T getParameter(std::string pluginName,
                       sdf::ElementPtr element, std::string parameter_name,
                       std::string dimension, T default_value) {
            gzmsg << pluginName << ": " << parameter_name;
            if (element->HasElement(parameter_name.c_str())) {
                T var = element->Get<T>(parameter_name.c_str());
                gzmsg << "=" << var << " " << dimension  << std::endl;
                return var;
            } else {
                gzmsg << " using default " << default_value
                      << " " << dimension  << std::endl;
                return default_value;
            }
        }
    }

}

#endif
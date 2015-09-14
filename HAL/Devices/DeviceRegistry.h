#pragma once

#include <HAL/config.h>
#include <HAL/Utils/Uri.h>

#include <memory>
#include <map>

namespace hal
{

  // Forward declaration
  template<typename BaseDevice> class DriverFactory;

  template<typename BaseDevice>
    class DeviceRegistry
    {
      public:
        /// global device registery singleton 
        static DeviceRegistry<BaseDevice>& Instance();

        DeviceRegistry();
        ~DeviceRegistry();

        /// Register factory pointer accessible by device_name
        void RegisterFactory(
            const std::string& device_name,
            DriverFactory<BaseDevice>* factory
            );

        // Register device 'name' as alias for 'alias'
        void RegisterAlias(
            const std::string& name,
            const std::string& alias
            );

        // Get factory associated with uri
        std::shared_ptr<BaseDevice> Create(const Uri& uri);

        void Destroy(BaseDevice* dev);

        // print the map<string,Dev> table 
        void ListDrivers();

      protected:

        // Map of driver names to aliases.
        std::map<std::string,std::string> m_aliases;

        // Map of driver name -> driver factory instance
        std::map<std::string,DriverFactory<BaseDevice>*> m_factories;
    };

}

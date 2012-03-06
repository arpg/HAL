/*
    Drivers MUST register themselves by instantiating a global
    CarDriverRegisteryEntry singleton.
 */

#ifndef CAR_DRIVER_REGISTERY_H
#define CAR_DRIVER_REGISTERY_H

CarDriverInterface* CreateDriver( const std::string& sDriverName );

extern std::map<std::string,CarDriverInterface*(*)()> g_mDriverTable;

class CarDriverInterface;

template <class T>
class CarDriverRegisteryEntry
{
    public:
        //////////////////////////////////////////////////////////////////////
        // Generic creation function
        static T* DriverCreationFunction()
        {
            return new T;
        }

        //////////////////////////////////////////////////////////////////////
        // Register a driver
        CarDriverRegisteryEntry(
                const std::string& sDriverName
                )
        {
//            std::cout << "Registered '" << sDriverName << "' driver.\n";
//            printf("g_mDriverTable lives at %p\n", &g_mDriverTable );

            g_mDriverTable[sDriverName] 
                = (CarDriverInterface* (*)())DriverCreationFunction;
//            std::cout << "Known:\n";
//            std::map<std::string,CarDriverInterface*(*)()>::iterator it;
//            for( it = g_mDriverTable.begin(); it != g_mDriverTable.end(); it++ ){
//                std::cerr << "\t" << it->first << std::endl;
//            }
        }
};

#endif	// CAR_DRIVER_REGISTERY_H


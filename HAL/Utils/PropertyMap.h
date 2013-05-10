/*
   \file PropertyMap

   Simple map of string strings allow storing any key-value pair where the
   value class supports << >> operators.

   This is header only implementation, so you can copy it around at will.

*/

#ifndef _PROPERTY_MAP_
#define _PROPERTY_MAP_

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <limits>

///////////////////////////////////////////////////////////////////////////////
// Helper class to endow objects with generic "properties"
class PropertyMap
{
    public:
        ////////////////////////////////////////////////////////////////////////
        template<class T>
        friend void output( T& oStream, const PropertyMap& prop ) {
            std::map<std::string,std::string>::const_iterator it;
            for( it = prop.m_mPropertyMap.begin(); it != prop.m_mPropertyMap.end(); it++ ){
                //oStream << it->first << " : " << it->second << "\n";
                oStream << it->first << it->second;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        friend std::ostream& operator<<( std::ostream& oStream, const PropertyMap& prop ) {
            std::map<std::string,std::string>::const_iterator it;
            for( it = prop.m_mPropertyMap.begin(); it != prop.m_mPropertyMap.end(); it++ ){
                oStream << it->first << it->second;
            }
            return oStream;
        }

        ////////////////////////////////////////////////////////////////////////
        void PrintPropertyMap()
        {
            output( std::cout, *this );
        }

        ////////////////////////////////////////////////////////////////////////
        std::string GetProperty(
                const std::string& sPropertyName,
                const std::string& sDefaultValue = ""
                )
        {
            std::map<std::string,std::string>::iterator it;
            it = m_mPropertyMap.find( sPropertyName );
            if( it == m_mPropertyMap.end() ){
                return sDefaultValue;
            }
            return it->second;
        }

        ////////////////////////////////////////////////////////////////////////
        template <class T> T GetProperty(
                const std::string& sPropertyName,
                const T& tDefaultValue = T()
                )
        {
            std::map<std::string,std::string>::iterator it;
            it = m_mPropertyMap.find( sPropertyName );
            if( it == m_mPropertyMap.end() ){
                return tDefaultValue;
            }
            T t;
            StrToVal( t, it->second );
            return t;
        }

        ////////////////////////////////////////////////////////////////////////
        template <class T>
            void SetProperty(
                    const std::string& sPropertyName,
                    T tDesiredValue
                    )
            {
                m_mPropertyMap[ sPropertyName ] = ValToStr( tDesiredValue );
            }

         ////////////////////////////////////////////////////////////////////////
         std::map<std::string,std::string>& GetPropertyMap()
         {
            return          m_mPropertyMap;
         }

    private:

        ////////////////////////////////////////////////////////////////////////
        template <class T>
            void StrToVal( T& t, const std::string& sValue )
            {
                std::istringstream iss( sValue );
                iss >> t;
            }

        ////////////////////////////////////////////////////////////////////////
        template <class T>
            std::string ValToStr( const T& t )
            {
                std::ostringstream oss;
                oss << t;
                return oss.str();
            }

        ////////////////////////////////////////////////////////////////////////
        // Specialization for double for maximum precision
        std::string ValToStr( const double& val )
        {
            std::ostringstream oss;
            oss.precision( std::numeric_limits<double>::digits10 + 1 );
            oss << std::fixed << std::scientific << val;
            return oss.str();
        }

        ////////////////////////////////////////////////////////////////////////
        // Specialization for float for maximum precision
        std::string ValToStr( const float& val )
        {
            std::ostringstream oss;
            oss.precision( std::numeric_limits<float>::digits10 + 1 );
            oss << std::fixed << std::scientific << val;
            return oss.str();
        }

        std::map<std::string,std::string>   m_mPropertyMap;
};

#endif


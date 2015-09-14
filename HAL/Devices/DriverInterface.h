#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <limits>

#include <HAL/Utils/PropertyMap.h>

namespace hal 
{
  // Top level device driver class s.t. all drivers have properties that can be set/get.
  class DriverInterface 
  {
    public:

      template<typename T>
        void SetProperty(const std::string& key, T value)
        {   
          driver_properties_.SetProperty(key,value);
        }

      template<typename T>
        T GetProperty(const std::string& key, T default_val = T() ) const
        {
          return driver_properties_.GetProperty(key,default_val);
        }


      void SetDefaultProperties( const std::vector<param_t>& params )
      {
         driver_properties_.SetProperties(params);
      }

      bool ParseUriProperties( const PropertyMap& m )
      {
        for( auto& it : m.GetPropertyMap() ){
          if( !driver_properties_.Contains( it.first ) ){
            std::cerr << "Unkown property: '" << it.first << "'\n";
            return false;
          }
        }
        // OK all properties are known, so just copy them in.
        driver_properties_.SetProperties( m );
        return true;
      }

      void PrintPropertyMap()
      {
        driver_properties_.PrintPropertyMap();
      }


    protected:
      PropertyMap driver_properties_;

/*
    struct Param
    {
      std::string param;
      std::string defaultval;
      std::string help;
    };

    inline std::vector<Param>& Params() 
    {
      return params_;
    }


    protected:
    std::vector<Param> params_;
    public:
        ////////////////////////////////////////////////////////////////////////
        template<class T>
        friend void output( T& oStream, const DriverInterface& prop ) {
            std::map<std::string,std::string>::const_iterator it;
            for( it = prop.m_mPropertyMap.begin(); it != prop.m_mPropertyMap.end(); it++ ){
                //oStream << it->first << " : " << it->second << "\n";
                oStream << it->first << it->second;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        friend std::ostream& operator<<( std::ostream& oStream, const DriverInterface& prop ) {
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
        */
  };

}


/*
   \file PropertyMap

   Simple map of string strings allow storing any key-value pair where the
   value class supports << >> operators.

   Each property is tuple with {key,value,help} strings.

*/

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <limits>

struct param_t
{
  std::string key;
  std::string value;
  std::string desc;
};


///////////////////////////////////////////////////////////////////////////////
// Helper class to endow objects with generic "properties"
class PropertyMap
{
    public:


      ////////////////////////////////////////////////////////////////////////
      template<class T>
        friend void output( T& oStream, const PropertyMap& prop ) {
          std::map<std::string,param_t>::const_iterator it;
          for( it = prop.property_map_.begin(); it != prop.property_map_.end(); it++ ){
            oStream << it->first << it->second.value;
          }
        }

      ////////////////////////////////////////////////////////////////////////
      friend std::ostream& operator<<( std::ostream& oStream, const PropertyMap& prop ) {
        std::map<std::string,param_t>::const_iterator it;
        for( it = prop.property_map_.begin(); it != prop.property_map_.end(); it++ ){
          oStream << it->first << it->second.value;
        }
        return oStream;
      }

      ////////////////////////////////////////////////////////////////////////
      PropertyMap()
      {
      }

      ////////////////////////////////////////////////////////////////////////
      void SetProperties( const std::vector<param_t>& params )
      {
        for( auto& it : params ){
          property_map_[it.key] = it;
        }
      }

      ////////////////////////////////////////////////////////////////////////
      void SetProperties( const std::map<std::string,param_t>& m )
      {
        for( auto& it : m ){
          property_map_[it.first] = it.second;
        }
      }

      ////////////////////////////////////////////////////////////////////////
      void SetProperties( const PropertyMap& m )
      {
        for( auto& it : m.property_map_ ){
          property_map_[it.first] = it.second;
        }
      }

      ////////////////////////////////////////////////////////////////////////
      void PrintPropertyMap()
      {
        //output( std::cout, *this );
        std::map<std::string,param_t>::const_iterator it;
        for( auto& it : property_map_ ){
          std::cout << "  " << it.first << ",   default value: " << it.second.value << ",   desc: " << it.second.desc << "\n";
        }
      }

      ////////////////////////////////////////////////////////////////////////
      std::string GetProperty(
          const std::string& property_name,
          const std::string& default_value = ""
          ) const
      {
        auto it = property_map_.find( property_name );
        if( it == property_map_.end() ){
          return default_value;
        }
        return it->second.value;
      }

      ////////////////////////////////////////////////////////////////////////
      template <class T> T GetProperty(
          const std::string& property_name,
          const T& default_value = T()
          ) const
      {
        auto it = property_map_.find( property_name );
        if( it == property_map_.end() ){
          return default_value;
        }
        T t;
        StrToVal( t, it->second.value );
        return t;
      }

      ////////////////////////////////////////////////////////////////////////
      template <class T>
        void SetProperty(
            const std::string& key,
            const T value,
            const std::string& description = ""
            )
        {
          property_map_[ key ] = {key, ValToStr<T>(value), description};
        }

      bool Contains( const std::string& key ) const
      {
        auto it = property_map_.find( key );
        return it == property_map_.end() ? false : true; 
      }

      ////////////////////////////////////////////////////////////////////////
      const std::map<std::string,param_t>& GetPropertyMap() const
      {
        return property_map_;
      }

    private:

      ////////////////////////////////////////////////////////////////////////
      template <class T>
        void StrToVal( T& t, const std::string& value ) const
        {
          std::istringstream iss( value );
          iss >> t;
        }

      ////////////////////////////////////////////////////////////////////////
      template <class T>
        std::string ValToStr( const T& t ) const
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
    protected:
      std::map<std::string,param_t>   property_map_;
};



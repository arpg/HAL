#pragma once

#include <exception>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <set>

#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

#include <memory>

#include <HAL/Utils/PropertyMap.h>
#include <HAL/Utils/StringUtils.h>

namespace hal
{

  class Uri
  {
    public:
      Uri()
      {
      }

      // Copy constructor
      Uri(const Uri &rhs)
      {
        this->properties = rhs.properties;
        this->scheme = rhs.scheme;
        this->url  = rhs.url;
      }

      Uri(const std::string& str_uri)
      {
        // Find Scheme delimiter
        size_t ns = str_uri.find_first_of(':');
        if( ns != std::string::npos ){
          scheme = str_uri.substr(0,ns);

          // Find url delimiter
          size_t nurl = str_uri.find("//",ns+1);
          if(nurl != std::string::npos){
            // If there is space between the delimiters, extract protocol arguments
            if( nurl-ns > 1){
              if( str_uri[ns+1] == '[' && str_uri[nurl-1] == ']' ){
                std::string queries = str_uri.substr(ns+2, nurl-1 - (ns+2) );
                std::vector<std::string> params = Split(queries, ',');
                for(const std::string& p : params){
                  std::vector<std::string> args = Split(p, '=');
                  std::string key = args[0];
                  std::string val = args.size() > 1 ? args[1] : "";
                  Trim(key);
                  Trim(val);
                  properties.SetProperty(key, val);
                }
              }else{
                //throw DeviceException("Bad video URI", str_uri);
                printf( "Uri::Uri() Bad video URI: %s\n", str_uri.c_str() );
                return;
              }
            }

            url = str_uri.substr(nurl+2);
          }
        }else{
          scheme = str_uri;
          url = str_uri;
        }
      }

      void SetProperties( const PropertyMap& m )
      {
        properties.SetProperties(m);
      }

      std::string scheme;
      std::string url;
      PropertyMap properties;
  };


  struct ImageDim
  {
    inline ImageDim() : x(0), y(0) {}
    inline ImageDim(size_t x, size_t y) : x(x), y(y) {}
    size_t x;
    size_t y;
  };

  struct ImageRoi
  {
    inline ImageRoi() : x(0), y(0), w(0), h(0) {}
    inline ImageRoi(size_t x, size_t y, size_t w, size_t h) : x(x), y(y), w(w), h(h) {}
    size_t x; size_t y;
    size_t w; size_t h;
  };

  inline std::istream& operator>> (std::istream &is, ImageDim &dim)
  {
    is >> dim.x; is.get(); is >> dim.y;
    return is;
  }

  inline std::istream& operator>> (std::istream &is, ImageRoi &roi)
  {
    is >> roi.x; is.get(); is >> roi.y; is.get();
    is >> roi.w; is.get(); is >> roi.h;
    return is;
  }


}

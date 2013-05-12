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

#include <HAL/Utils/StringUtils.h>
#include <HAL/Camera/CameraException.h>

namespace hal
{

class PropertyMap
{
public:
    bool Contains(const std::string& key) const
    {
        return params.find(key) != params.end();
    }
    
    template<typename T>
    void Set(const std::string& key, T value)
    {
        params[key] = ValToStr<T>(value);
    }

    template<typename T>
    T Get(const std::string& key, T default_val) const
    {
        std::map<std::string,std::string>::const_iterator v = params.find(key);
        if(v != params.end()) {
            return StrToVal<T>(v->second);
        }else{
            return default_val;
        }
    }
protected:
    std::map<std::string,std::string> params;    
};

class Uri
{
public:
    Uri()
    {    
    }

    Uri(std::string str_uri)
    {
        // Find Scheme delimiter
        size_t ns = str_uri.find_first_of(':');
        if( ns != std::string::npos )
        {
            scheme = str_uri.substr(0,ns);
            
            // Find url delimiter
            size_t nurl = str_uri.find("//",ns+1);
            if(nurl != std::string::npos)
            {
                // If there is space between the delimiters, extract protocol arguments
                if( nurl-ns > 1)
                {
                    if( str_uri[ns+1] == '[' && str_uri[nurl-1] == ']' )
                    {
                        std::string queries = str_uri.substr(ns+2, nurl-1 - (ns+2) );
                        std::vector<std::string> params = split(queries, ',');
                        for(const std::string& p : params)
                        {
                            std::vector<std::string> args = split(p, '=');
                            std::string key = args[0];
                            std::string val = args.size() > 1 ? args[1] : "";
                            trim(key);
                            trim(val);
                            properties.Set(key, val);
                        }
                    }else{
                        throw VideoException("Bad video URI");
                    }
                }
                
                url = str_uri.substr(nurl+2);
            }
        }else{
            scheme = "file";
            url = str_uri;
        }
    }

    std::string scheme;
    std::string url;
    PropertyMap properties;
};

}

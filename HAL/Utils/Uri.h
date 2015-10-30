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
#include <HAL/Devices/DeviceException.h>

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

    std::ostream& Print( std::ostream& os, char keyValDelim='=', char itemDelim=',' ) const
    {
        if( params.empty() ) {
            return os;
        }

        std::map<std::string,std::string>::const_iterator i = params.begin();

        if(i!=params.end()) {
            os << i->first << keyValDelim << i->second;
        }

        for(++i; i!=params.end(); ++i)
        {
            os << itemDelim;
            os << i->first << keyValDelim << i->second;
        }
        return os;
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

    Uri(const std::string& str_uri)
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
//                        SetProperties( queries );
                        /* */
                        std::vector<std::string> params = Split(queries, ',');
                        for(const std::string& p : params)
                        {
                            std::vector<std::string> args = Split(p, '=');
                            std::string key = args[0];
                            std::string val = args.size() > 1 ? args[1] : "";
                            Trim(key);
                            Trim(val);
                            properties.Set(key, val);
                        }
                        /* */
                    }else{
                        throw DeviceException("Bad video URI", str_uri);
                    }
                }

                url = str_uri.substr(nurl+2);
            }
        }else{
            scheme = "";
            url = str_uri;
        }
    }

    void Print( std::ostream& os ) const
    {
        os << scheme << ":[";
        properties.Print(os, '=', ',');
        os << "]//" << url;
    }

    std::string PrintProperties() const
    {
        std::ostringstream oss;
        properties.Print(oss);
        return oss.str();
    }

    std::string ToString() const
    {
        std::ostringstream oss;
        Print(oss);
        return oss.str();
    }

    void SetProperties( const std::string& props )
    {
        std::vector<std::string> params = Split(props, ',');
        for(const std::string& p : params)
        {
            std::vector<std::string> args = Split(p, '=');
            std::string key = args[0];
            std::string val = args.size() > 1 ? args[1] : "";
            Trim(key);
            Trim(val);
            properties.Set(key, val);
        }
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

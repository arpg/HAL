#pragma once

#include <dirent.h>
#include <sys/stat.h>

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <list>

namespace hal
{

inline std::string& LTrim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

inline std::string& RTrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

inline std::string& Trim(std::string& s)
{
    return LTrim(RTrim(s));
}

inline std::vector<std::string>& Split(const std::string& s, char delim, std::vector<std::string>& elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline std::vector<std::string> Split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return Split(s, delim, elems);
}

inline std::vector<std::string> Expand(const std::string &s, char open='[', char close=']', char delim=',')
{
    const size_t no = s.find_first_of(open);
    if(no != std::string::npos) {
        const size_t nc = s.find_first_of(close, no);
        if(no != std::string::npos) {
            const std::string pre  = s.substr(0, no);
            const std::string mid  = s.substr(no+1, nc-no-1);
            const std::string post = s.substr(nc+1, std::string::npos);
            const std::vector<std::string> options = Split(mid, delim);
            std::vector<std::string> expansion;
            for(const std::string& op : options) {
                std::string full = pre + op + post;
                expansion.push_back(full);
            }
            return expansion;
        }
        // Open but no close is unusual. Leave it for caller to see if valid
    }
    return {s};
}

inline std::string DirUp(const std::string& dir)
{
    const size_t nLastSlash = dir.find_last_of('/');

    if(nLastSlash != std::string::npos) {
        return dir.substr(0, nLastSlash);
    }else{
        return std::string();
    }
}

inline bool FileExists(const std::string& filename)
{
    struct stat buf;
    return stat(filename.c_str(), &buf) != -1;
}

inline bool IsDir(const std::string& filename)
{
    struct stat buf;
    if(0 == stat(filename.c_str(), &buf)) {
        return S_ISDIR(buf.st_mode);
    }
    return false;
}

inline std::string ExpandTildePath(const std::string& sPath)
{
    if(sPath.length() >0 && sPath[0] == '~') {
        const char* sHomeDir = getenv("HOME");
        return std::string(sHomeDir) + sPath.substr(1,std::string::npos);
    }else{
        return sPath;
    }
}

// Based on http://www.codeproject.com/Articles/188256/A-Simple-Wildcard-Matching-Function
inline bool WildcardMatch(const std::string& query, const std::string& wildcard)
{
    const char* psQuery = query.c_str();
    const char* psWildcard = wildcard.c_str();

    while(*psWildcard)
    {
        if(*psWildcard=='?')
        {
            if(!*psQuery)
                return false;

            ++psQuery;
            ++psWildcard;
        }
        else if(*psWildcard=='*')
        {
            if(WildcardMatch(psQuery,psWildcard+1))
                return true;

            if(*psQuery && WildcardMatch(psQuery+1,psWildcard))
                return true;

            return false;
        }
        else
        {
            if(*psQuery++ != *psWildcard++ )
                return false;
        }
    }

    return !*psQuery && !*psWildcard;
}

inline bool WildcardFileList(const std::string& wildcard, std::vector<std::string>& file_vec)
{
    size_t nLastSlash = wildcard.find_last_of('/');

    std::string sPath;
    std::string sFileWc;

    if(nLastSlash != std::string::npos) {
        sPath =   wildcard.substr(0, nLastSlash);
        sFileWc = wildcard.substr(nLastSlash+1, std::string::npos);
    }else{
        sPath = ".";
        sFileWc = wildcard;
    }

    sPath = ExpandTildePath(sPath);

    struct dirent **namelist;
    int n = scandir(sPath.c_str(), &namelist, 0, alphasort ); // sort alpha-numeric
    if (n >= 0){
        std::list<std::string> file_list;
        while( n-- ){
            const std::string sName(namelist[n]->d_name);
            if( sName != "." && sName != ".." && WildcardMatch(sName, sFileWc) ) {
                const std::string sFullName = sPath + "/" + sName;
                file_list.push_front( sFullName );
            }
            free(namelist[n]);
        }
        free(namelist);

        file_vec.reserve(file_list.size());
        file_vec.insert(file_vec.begin(), file_list.begin(), file_list.end());
        return file_vec.size() > 0;
    }
    return false;
}

template <class T> inline
void StrToVal( T& t, const std::string& sValue )
{
    std::istringstream iss( sValue );
    iss >> t;
}

template <class T> inline
T StrToVal( const std::string& sValue )
{
    T t;
    std::istringstream iss( sValue );
    iss >> t;
    return t;
}

template <class T> inline
std::string ValToStr( const T& t )
{
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

}

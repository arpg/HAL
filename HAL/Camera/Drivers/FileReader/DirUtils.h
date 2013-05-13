#pragma once

#include <string>
#include <vector>
#include <list>
#include <sstream>

namespace hal
{
    ////////////////////////////////////////////////////////////////////////////////////////
    /// Find regular expression in a file.
    std::string GrepStr(
            const std::string& sBuf,
            const std::string& sRegex
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Find regular expression in a file.
    std::string Grep(
            const std::string& sFile,
            const std::string& sRegex
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool MoveFile(
            const std::string& sSrc, 
            const std::string& sDst
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> SplitString( 
            const std::string& sStr, 
            const std::string& sDelimiters 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    std::string StringReplace(
            const std::string& sStr,
            const std::string& sTok,
            const std::string& sReplace
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool RealPath(
            const std::string& sPath,
            std::string& sFullPath 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    std::string RealPath(
            const std::string& sPath
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool IsDirectory(
            const std::string& sDir
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool IsFile( 
            const std::string& sFile
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool CreateMissingDirectory( 
            const std::string& sDir
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    //// Split a fully qualified path into parts -path, filestem and extension.
    std::string StripPath(
            std::string sFullPath     //< Input
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    std::string FileExtension(
            const std::string& sFile 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    //// Split a fully qualified path into parts -path, filestem and extension.
    bool GetFileParts(
            std::string sFullPath,     //< Input
            std::string& sPath,       //< Output
            std::string& sFile,        //< Output
            std::string& sExtension   //< Output
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get full directory listing, without caching.  Filenames include the full path.
    bool ScanDir(
            const std::string & sPath,          //< Input: Directory to scan.
            std::list<std::string> &sContents   //< Output: List of directory.
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Generate a cache filename in /tmp.
    std::string MakeCacheFilename(
            const std::string& sDir 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool ATime(
            const std::string& sFile,
            double &dATime
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool MTime(
            const std::string& sFile, 
            double &dMTime
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    bool CTime(
            const std::string& sFile,
            double &dCTime 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    // Return directory listing, reading results from a cache file if it exists, and creating that cache otherwise.
    bool GetCachedDirectoryContents(
            const std::string & sPath,         //< Input: Directory to scan.
            std::list<std::string> &lContents  //< Output: list of files found.
            );
    
    ////////////////////////////////////////////////////////////////////////////////////////
    std::string ToLower( 
            const std::string& str
            );
    
    ////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find the first file matching the given regular expression. 
    bool FindFile(
            const std::string& sDir,
            const std::string& sRegex,
            std::string& sFile,
            int nLevel = 0
            );

    // Recursively find all files in and below sDir that match the given regular expression.
    bool RecursivelyFindFiles(
            const std::string& sDir,
            const std::string& sRegEx,
            std::list<std::string>& vFileList,
            int nMaxDepth = 10,
            int nDepth = 0
            );
 

    // Recursively find all files in and below sDir
    bool FindFiles( 
            const std::string& sRegex, 
            std::vector<std::string>& vFiles,
            bool bMatchCase = true 
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find all files in and below sDir
    bool RecursivelyFindFiles(
            const std::string sDir,
            std::list<std::string>& vFileList,
            std::vector<std::string> vSkipDirs,
            int nMaxDepth,
            int nDepth
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get an element attribute out of an XML file.
    bool GetAttrib( 
            const std::string& sFile,
            const std::string& sElement,
            const std::string& sAttribName,
            std::string& sRes
            );

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get an element attribute out of an XML file.
    template <class T> 
    inline bool GetAttrib( 
            const std::string& sFile,
            const std::string& sElement,
            const std::string& sAttribName,
            T& t
            )
    {
        std::string sRes;
        if( GetAttrib( sFile, sElement, sAttribName, sRes ) ){
            std::stringstream ss;
            ss << sRes;
            ss >> t;
            return true;
        }
        return false;
    }
}

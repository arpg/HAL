#include "DirUtils.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include <cctype>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <list>

#include <regex>
#include <limits.h>

namespace hal
{
    int PrintHandlerGetErrorLevel()
    {
        return 0;
    }

    void PrintMessage( int nVerbosityLevel, const char *sMsg, ... )
    {
        if( nVerbosityLevel <= PrintHandlerGetErrorLevel() ){
            if( sMsg == NULL ){
                return;
            }
    
            char *pBuffer = NULL;
            va_list ap;
            memset( &ap, 0, sizeof( ap ) );
    
            va_start( ap, sMsg );
            {
                int nResult = INT_MAX;
                int nLength = 2048;
                while( nResult >= nLength ) {
                    if( pBuffer != NULL ) {
                        delete[] pBuffer;
                    }
                    nLength *= 2;
                    pBuffer = new char[nLength + 1];
                    memset( pBuffer, 0, nLength + 1 );
                    nResult = vsnprintf( pBuffer, nLength, sMsg, ap );
                }
            }
            va_end( ap );
    
            fputs( pBuffer, stdout );
            fflush( stdout );
            delete[] pBuffer;
        }
    }
    
    void PrintError( const char *sMsg, ... )
    {
        if( sMsg == NULL ){
            return;
        }
    
        char *pBuffer = NULL;
        va_list ap;
        memset( &ap, 0, sizeof( ap ) );
        va_start( ap, sMsg );
        {
            int nResult = INT_MAX;
            int nLength = 2048;
            while( nResult >= nLength ) {
                nLength *= 2;
                if( pBuffer != NULL ) {
                    delete[] pBuffer;
                }
                pBuffer = new char[nLength + 1];
                memset( pBuffer, 0, nLength + 1 );
                nResult = vsnprintf( pBuffer, nLength, sMsg, ap );
            }
        }
        va_end( ap );
    
        fputs( pBuffer, stderr );
        fflush( stderr );
        delete[] pBuffer;
    }


    ////////////////////////////////////////////////////////////////////////////////////////
    /// Find regular expression in a file.
    std::string GrepStr(
            const std::string& sBuf,
            const std::string& sRegex
            )
    {
        std::regex ex( sRegex );
        std::cmatch what;
        if( std::regex_search( sBuf.c_str(), what, ex ) ){
            return std::string(what[0].first, what[0].second );
        }
        return "";
    }
    

    std::vector<std::string> SplitString( const std::string& sStr, const std::string& sDelimiters )
    {
        std::vector< std::string > vWords;

        // Skip delimiters at beginning.
        std::string::size_type nLastPos = sStr.find_first_not_of( sDelimiters, 0 );

        // Find first "non-delimiter".
        std::string::size_type nPos     = sStr.find_first_of( sDelimiters, nLastPos );

        while( std::string::npos != nPos || std::string::npos != nLastPos){
            // Found a token, add it to the vector.
            vWords.push_back( sStr.substr( nLastPos, nPos - nLastPos ) );
            // Skip delimiters.  Note the "not_of"
            nLastPos = sStr.find_first_not_of( sDelimiters, nPos );
            // Find next "non-delimiter"
            nPos = sStr.find_first_of( sDelimiters, nLastPos );
        }
        return vWords;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string StringReplace( const std::string& sStr,const std::string& sTok, const std::string& sReplace )
    {
        std::string sResult = sStr;
        while( sResult.find(sTok) != std::string::npos ){
            sResult.replace( sResult.find(sTok), sTok.length(), sReplace );        
        }
        return sResult;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string RealPath( const std::string& sFile )
    {
        std::string sPath; 
        if( !RealPath( sFile, sPath ) ){
            return "";
        }
        return sPath;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool RealPath( const std::string& sPath, std::string& sRealPath )
    {
#ifdef PATH_MAX
        int path_max = PATH_MAX;
#else
        int path_max = pathconf(path, _PC_PATH_MAX);
        if (path_max <= 0){
            path_max = 4096;
        }
#endif
        char *resolved_path = (char*)malloc( path_max );
        bool retVal = true;
        if( realpath( sPath.c_str(), resolved_path ) == NULL ) {
            retVal = false;
        }
        else {
            sRealPath = resolved_path;
        }
        free( resolved_path );
        return retVal;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool IsDirectory( const std::string& sDir )
    {
        struct stat StatBuf;
        if( stat( sDir.c_str(), &StatBuf ) < 0 ){
            return false;
        }
        return S_ISDIR(StatBuf.st_mode);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool IsFile( const std::string& sFile )
    {
        struct stat StatBuf;
        if( stat( sFile.c_str(), &StatBuf ) < 0 ){
            return false;
        }
        return S_ISREG( StatBuf.st_mode );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool CreateMissingDirectory( const std::string& sDir )
    {
        if( hal::IsDirectory( sDir ) == false ){
            if( mkdir( sDir.c_str(), S_IRWXU ) < 0 ){
                PrintError( "ERROR: creating \"%s\" -- %s\n", sDir.c_str(), strerror( errno ) );
                return false;
            }
            return true;
        }
        chmod( sDir.c_str(), S_IRWXU );
        PrintMessage( 2, "%s already exists\n", sDir.c_str() );
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Split a fully qualified path into parts -path, filestem and extension.
    std::string StripPath(
            std::string sFullPath     //< Input
            )
    {
        // split on "." for extension if it is there.
        size_t ii = sFullPath.find_last_of( "/\\" );
        if( ii != std::string::npos ){
            return sFullPath.substr( ii+1, sFullPath.size()-ii );
        }
        return sFullPath; 
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string FileExtension( const std::string& sFile )
    {
        size_t n = sFile.find_last_of( "." );
        if( n != std::string::npos ){
            return sFile.substr(n+1);
        }
        return "";
    } 

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Split a fully qualified path into parts -path, filestem and extension.
    bool GetFileParts(
            std::string sFullPath,     //< Input
            std::string& sPath,       //< Output
            std::string& sFile,        //< Output
            std::string& sExtension   //< Output
            )
    {

        size_t nFS = sFullPath.find_last_of("/");
        size_t nBreak = nFS;

        std::string sFullFile;
        if( nBreak==std::string::npos ){
            //there is no path -- must be cwd
            char resolved_path[1024];
            if( realpath( "./", resolved_path ) == NULL ){
                PrintError("ERROR: realpath() -- %s\n", strerror(errno) );
                return false;
            }
            sPath = resolved_path;
            sFullFile = sFullPath;
        }
        else{
            // split path and file
            sPath = sFullPath.substr( 0, nBreak );
            sFullFile = sFullPath.substr( nBreak+1 );
        }

        // split on "." for extension if it is there.
        size_t ii = sFullFile.find_last_of( "." );
        if( ii != std::string::npos ){
            sFile = sFullFile.substr( 0, ii );
            sExtension = sFullFile.substr( ii+1, sFullFile.size()-ii );
        }
        else{
            sFile = sFullFile;
            sExtension = "";
        }

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Get full directory listing, without caching.  Filenames include the full path.
    bool ScanDir(
            const std::string & sPath,          //< Input: Directory to scan.
            std::list<std::string> &sContents   //< Output: List of directory.
            )
    {
        struct dirent **namelist;
        int n = scandir(sPath.c_str(), &namelist, 0, alphasort );
        if (n < 0){
            PrintMessage( 0, "ERROR: reading directory contents %s\n",strerror(errno));
            return false;
        }
        while( n-- ){
            std::string sName(namelist[n]->d_name);
            std::string sFullName = sPath[sPath.length()-1] == '/' ? sPath + sName : sPath + "/" + sName;
            if( sName != "." && sName != ".." ){
                sContents.push_front( sFullName );
            }
            free(namelist[n]);
        }
        free(namelist);
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Generate a cache filename in /tmp.
    std::string MakeCacheFilename( const std::string& sDir )
    {
        std::string sCacheFilename;
        if( sDir[sDir.length()-1] == '/' ){
            sCacheFilename = sDir + "libMiscDirCache_V3";
        }
        else{
            sCacheFilename = sDir + "/libMiscDirCache_V3";
        }

        sCacheFilename = "/tmp/" + StringReplace( sCacheFilename, "/", "." );
        //    sCacheFilename = "/tmp" +  sCacheFilename;
        return sCacheFilename;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ATime( const std::string& sFile, double &dATime )
    {
        struct stat filestat;
        if( stat( sFile.c_str(), &filestat ) < 0 ){
            return false;
        }
#ifdef PLATOFRM_DARWIN
        dATime = filestat.st_atimespec.tv_sec + 1e-9*filestat.st_atimespec.tv_nsec;
#elif PLATFORM_LINUX
        dATime = filestat.st_atim.tv_sec + 1e-9*filestat.st_atim.tv_nsec;
#else
        dATime = filestat.st_atime;
#endif
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool MTime( const std::string& sFile, double &dMTime )
    {
        struct stat filestat;
        if( stat( sFile.c_str(), &filestat ) ){
            return false;
        }
#ifdef PLATOFRM_DARWIN
        dMTime = filestat.st_mtimespec.tv_sec + 1e-9*filestat.st_mtimespec.tv_nsec;
#elif PLATFORM_LINUX
        dMTime = filestat.st_mtim.tv_sec + 1e-9*filestat.st_mtim.tv_nsec;
#else
        dMTime = filestat.st_mtime;
#endif
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool CTime( const std::string& sFile, double &dCTime )
    {
        struct stat filestat;
        if( stat( sFile.c_str(), &filestat ) ){
            return false;
        }
#ifdef PLATOFRM_DARWIN
        dCTime = filestat.st_ctimespec.tv_sec + 1e-9*filestat.st_ctimespec.tv_nsec;
#elif PLATFORM_LINUX
        dCTime = filestat.st_ctim.tv_sec + 1e-9*filestat.st_ctim.tv_nsec;
#else
        dCTime = filestat.st_ctime;
#endif
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Return directory listing, reading results from a cache file if it exists, and creating that cache otherwise.
    bool GetCachedDirectoryContents(
            const std::string & sPath,         //< Input: Directory to scan.
            std::list<std::string> &lContents  //< Output: list of files found.
            )
    {
        std::string sFullPath;
        if( !RealPath( sPath, sFullPath )){
            return false;
        }


        if( hal::IsDirectory(sFullPath) == false ){
            return false;
        }

        // First check if we've read this dir before. If you fiddle wiht this code, then you should change
        // the Version tag at the end of the cache file name so older cache files don't cause you trouble
        std::string sCacheFile = MakeCacheFilename( sFullPath );

        struct stat filestat;
        struct stat dirstat;
        bool bFileReadFromCache = false;

        if( stat( sPath.c_str(), &dirstat ) < 0 ){
            hal::PrintError("ERROR: could not get status of '%s' -- %s.\n", sPath.c_str(), strerror(errno));
            return false;
        }

        bool bKillFile = false;
        if( stat( sCacheFile.c_str(), &filestat ) == 0 ) { // sweet, we've cached the scandir results already
            if( filestat.st_mtime >= dirstat.st_mtime ){ // if the directory contents have not changed
                PrintMessage( 5, "Reading dir chache file %s\n", sCacheFile.c_str() ); 
                std::string sLine;
                std::ifstream infile( sCacheFile.c_str() );
                while( std::getline( infile, sLine ) ){
                    PrintMessage( 5, "    Read %s\n", sLine.c_str() );
                    lContents.push_back( sLine );
                }
                bFileReadFromCache = true;
            }
            else{
                PrintMessage( 5, "WARNING: directory '%s' cache out of date, rebuilding cache.\n", sPath.c_str() ); 
            }
        }
        else{
            PrintMessage( 1, "WARNING: Could not stat chache file '%s' -- %s.  rebuliding cache. \n", sCacheFile.c_str(), strerror(errno) ); 
        }

        if( bKillFile ){
            bFileReadFromCache = false;
            std::string cmd = "rm -f " + sCacheFile;
            if( system( cmd.c_str() ) == -1 ) {
                PrintMessage( 0, "ERROR: error removing cache file.\n");
            }
        }
        if( bFileReadFromCache == false ){
            PrintMessage( 5, "Reading from %s.  This may take a few seconds... ", sPath.c_str() );

            if( !hal::ScanDir( sFullPath, lContents )){
                PrintMessage( 5, "WARNING: Failed to open directory for reading");
                return false;
            }
            PrintMessage( 5, "done.\n");

            // save to cahce file 
            std::ofstream outfile( sCacheFile.c_str() );
            if( outfile.is_open() ){
                std::list< std::string >::iterator it;
                for( it = lContents.begin(); it != lContents.end(); it++ ){
                    outfile << *it << std::endl; 
                    PrintMessage( 5, "Saving '%s'\n", it->c_str() );
                }
                outfile.close();
            }
            else{
                PrintMessage( 0, "ERROR: Failed to save '%s' chache file\n", sCacheFile.c_str() ); 
            }
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string ToLower( const std::string& str )
    {
        std::string res = str;
        std::transform( res.begin(), res.end(), res.begin(), ::tolower );
        return res;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find the first file matching the given regular expression. 
    bool FindFile(
            const std::string& sDir,
            const std::string& sRegex,
            std::string& sFile,
            int nLevel
            )
    {
        std::regex ex( sRegex.c_str() );

        std::list<std::string> vThisDirFileList;
        if( !hal::GetCachedDirectoryContents( sDir, vThisDirFileList )){
            PrintMessage( 5, "WARNING: Failed to open directory for reading '%s' -- %s\n", sDir.c_str(), strerror(errno) );
            return false;
        }

        // Check this dir.
        PrintMessage( 3, "%sDir: '%s'\n", std::string(4*nLevel,' ').c_str(),  sDir.c_str() );
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){
            std::string sFilePart = StripPath( *it );
            if( std::regex_match( sFilePart, ex ) ){
                PrintMessage( 3, "%s '%s' Matched File: '%s'\n", 
                        std::string(4*(nLevel+1),' ').c_str(), sRegex.c_str(), sFilePart.c_str() );
                sFile = *it;
                return true;
            }
        }

        // now recursivey add files in sub-directories
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){            
            if( IsFile( *it ) ){
                continue;
            }
            std::string sNextDir = *it + "/";
            if( FindFile( sNextDir, sRegex, sFile, nLevel+1 ) ){
                return true;
            }
        }
        return false;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find all files in and below sDir that match the given regular expression.
    bool RecursivelyFindFiles(
            const std::string& sDir,
            const std::string& sRegEx,
            std::list<std::string>& vFileList,
            int nMaxDepth,
            int nDepth 
            )
    {
        std::regex ex(sRegEx);

        std::list<std::string> vThisDirFileList;

        if( nDepth > nMaxDepth ){
            return true;
        }

        std::string sFullPath;
        if( !RealPath( sDir, sFullPath )){
            return false;
        }

        if( !hal::GetCachedDirectoryContents( sFullPath, vThisDirFileList )){
            PrintMessage( 0, "ERROR: Failed to open directory for reading");
            return false;
        }

        // add full path info
        PrintMessage( 3, "%sDir: '%s'\n", std::string(4*nDepth,' ').c_str(), sDir.c_str() );
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){
            *it = sDir + *it;
            if( std::regex_match( *it, ex ) ){
                PrintMessage( 5, "%sMatching File: '%s'\n", 
                        std::string(4*(nDepth+1),' ').c_str(), StripPath(*it).c_str() );
                vFileList.push_back( *it );
            }
        }

        // now recursivey add files in sub-directories
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){            
            if( IsFile( *it ) ){
                continue;
            }
            std::string sNextDir = *it + "/";
            if( !hal::RecursivelyFindFiles( sNextDir, sRegEx, vFileList, nMaxDepth, nDepth+1 ) ){
                PrintMessage( 0, "ERROR: Failed to read directory '%s' -- %s", sNextDir.c_str(), strerror(errno) );
            }
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find all files in and below sDir
    bool FindFiles( 
            const std::string& sRegex, 
            std::vector<std::string>& vFiles,
            bool bMatchCase
            )
    {
        std::string sFileRegex = sRegex;
        
        // Split channel regex into directory and file components
        size_t pos = sFileRegex.rfind("/");
        std::string sDir = ".";

        if(pos != std::string::npos)
        {
            sDir = sFileRegex.substr(0,pos);
            sFileRegex = sFileRegex.substr(pos+1);
        }
        
        if( bMatchCase == false ){
            sFileRegex = ToLower( sFileRegex ); 
        }

        std::regex ex( sFileRegex.c_str() );
        std::list<std::string> vThisDirFileList;
        if( !hal::GetCachedDirectoryContents( sDir, vThisDirFileList )){
            return false;
        }

        // Check for matches.
        std::list<std::string>::iterator it;
        for( it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){
            std::string sFilePart = StripPath( *it );
            if( bMatchCase == false ){
                sFilePart = ToLower( sFilePart );
            }
            if( std::regex_match( sFilePart, ex ) ){
//                std::cout << "Matched: " << sFilePart << std::endl;
                vFiles.push_back( *it );
            }
        }
        return vFiles.size();
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Recursively find all files in and below sDir
    bool RecursivelyFindFiles(
            const std::string sDir,
            std::list<std::string>& vFileList,
            std::vector<std::string> vSkipDirs,
            int nMaxDepth,
            int nDepth
            )
    {
        std::list<std::string> vThisDirFileList;

        if( nDepth > nMaxDepth ){
            return true;
        }

        std::string sFullPath;
        if( !RealPath( sDir, sFullPath )){
            return false;
        }

        if( !hal::GetCachedDirectoryContents( sFullPath, vThisDirFileList )){
            PrintMessage( 0, "ERROR: Failed to open directory for reading");
            return false;
        }

        // add full path info
        PrintMessage( 3, "Dir: '%s'\n", sDir.c_str() );
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){
            PrintMessage( 5, "    File: '%s'\n", it->c_str() );
            *it = sDir + *it;
            vFileList.push_back( *it );
        }

        // now recursivey add files in sub-directories
        for( std::list<std::string>::iterator it = vThisDirFileList.begin(); it != vThisDirFileList.end(); it++ ){            
            if( IsFile( *it ) ){
                continue;
            }

            std::string sNextDir = *it + "/";

            // 
            bool bShouldSkipThisDir = false;
            for( size_t ii = 0; ii < vSkipDirs.size(); ii++ ){
                if( it->find( vSkipDirs[ii] ) != std::string::npos ){
                    PrintMessage( 3, "Skipping unwanted sub-dir '%s'\n", it->c_str() );
                    bShouldSkipThisDir = true;
                }
            }
            if( bShouldSkipThisDir == false ){
                PrintMessage( 3, "Recursing to Dir: '%s'\n", sNextDir.c_str() );
                if( !hal::RecursivelyFindFiles( sNextDir, vFileList, vSkipDirs, nMaxDepth, nDepth+1 ) ){
                    PrintMessage( 0, "ERROR: Failed to read directory '%s' -- %s", sNextDir.c_str(), strerror(errno) );
                }
            }
        }
        return true;
    }


}



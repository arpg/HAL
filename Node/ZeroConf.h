#ifndef _ZERO_CONF_H_
#define _ZERO_CONF_H_

#include <inttypes.h>
#include <errno.h>
#include <sys/types.h> // for u_char
#include <dns_sd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <stdio.h>

#include <boost/thread.hpp>

#include <string>
#include <vector>

struct ZeroConfRecord
{
    std::string     m_sServiceName;    //  service name
    std::string     m_sRegType; // _tcp or _udp
    std::string     m_sDomain;  // e.g. local
};

struct ZeroConfURL
{
    const char*  Host()
    {
        return m_sHost.c_str();
    }
    uint32_t Port()
    {
        return m_nPort;
    }

    std::string     m_sHost; // service host
    uint32_t        m_nPort; // service port
};

/// This class provides a *sychronous* interface to the local mdns service using
// Avahi's cross-platform Bounjour compatability API.
class ZeroConf
{

    public:
        ///////////////////////////////////////////////////////////////////////
        ZeroConf()
        {
            m_DNSServiceRef = 0;
        }

        ~ZeroConf()
        {
            m_ListenToServerThread.interrupt();
//            sleep(2);
//            m_ListenToServerThread.join();
//            printf("DNSServiceRefDeallocate( m_DNSServiceRef );\n");
//            DNSServiceRefDeallocate( m_DNSServiceRef );
        }

        ///////////////////////////////////////////////////////////////////////
        /// Generic interface to just register a service and bail
        bool RegisterService(
                const std::string& sName, //<
                const std::string& sRegType, //<
                uint16_t nPort,
                const std::string& sDomain = "local" //<
                )
        {
            DNSServiceErrorType err = kDNSServiceErr_NameConflict;
            std::string sServiceName = sName;
            //int nServiceCount = 0;
            while( err == kDNSServiceErr_NameConflict ){
                err = DNSServiceRegister(
                        &m_DNSServiceRef, // uninitialized DNSServiceRef, active till death
                        0, // nFlags
                        0, // interface index
                        sServiceName.c_str(),
                        sRegType.c_str(),// example "_ftp._tcp.". must be an
                        //  underscore that is followed by 1 to 14 characters that
                        // can be  letters, digits, or  hyphens.
                        sDomain.c_str(), // null, or maybe "local"
                        NULL, // host,  specifies the SRV target host name.
                        // Most applications  do  not  spefiy this
                        htons(nPort), // service port, in network byte order
                        0, // txtLen, length of txtRecord
                        NULL, // txtRecord, a properly  formatted  DNSTXT record
                        _RegisterServiceCallback, // DNSServiceServiceRegisterReply
                        &m_Record //
                        );

                // do renaming manually (since it doesn't work on linux)
                if( err == kDNSServiceErr_NameConflict) {
                    return false;
                    /*
                    printf("%s in use!\n", sServiceName.c_str() );
                    char buf[32];
                    snprintf( buf, 32, "%d", ++nServiceCount );
                    sServiceName = sName + "(" + buf + ")";
                    */
                }
            }

            if( err != kDNSServiceErr_NoError ){
                printf("ERROR calling DNSServiceRegister() -- erro %d\n", err);
                return false;
            }

            // ok now setup a thread that watches the avahi socket
            m_ListenToServerThread = boost::thread( _HandleEvents2, this );

            return true;
        }

        ///////////////////////////////////////////////////////////////////////
        /// Browse for a particular service type, e.g.m "_ssh._tcp"
        std::vector<ZeroConfRecord> BrowseForServiceType(
                const std::string& sServiceType,
                const std::string& sDomain = "local"
                )
        {
            std::vector<ZeroConfRecord> vRecords;

            DNSServiceRef ServiceRef;
            DNSServiceErrorType nErr =
                DNSServiceBrowse(
                        &ServiceRef,
                        0, // flags
                        0,  // interface index
                        sServiceType.c_str(), // service regtype
                        sDomain.c_str(), // domain
                        _BrowseReplyCallback,
                        &vRecords // user data
                        );
            if( nErr != kDNSServiceErr_NoError ){
                printf("error: BrowseForServiceType() -- %d\n", nErr );
            }

            // async interface:
            int dns_sd_fd  = DNSServiceRefSockFD(ServiceRef);
            //DNSServiceErrorType err;
            int nfds = dns_sd_fd + 1;
            fd_set readfds;
            struct timeval tv;

            // 1. Set up the fd_set as usual here.
            // This example client has no file descriptors of its own,
            // but a real application would call FD_SET to add them to the set here
            FD_ZERO(&readfds);

            // 2. Add the fd for our client(s) to the fd_set
            FD_SET(dns_sd_fd , &readfds);

            // 3. Set up the timeout.
            tv.tv_sec = 1;
            tv.tv_usec = 0;

            while (true){
                int result = select(nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv);
                if (result > 0){
                    DNSServiceErrorType err = kDNSServiceErr_NoError;
                    if( ServiceRef && 
                            FD_ISSET(dns_sd_fd, &readfds)) err = DNSServiceProcessResult(ServiceRef);
                    if( err != kDNSServiceErr_NoError) {
                        printf( "error looping %d\n", err);
                        return vRecords;
                    }
                } else if (result == 0)  {
                    //myTimerCallBack();
                    return vRecords;
                }   else {
                    if (errno != EINTR){
                        printf( "error %s\n", strerror(errno) );
                        return vRecords;
                    }
                }
            }

        /*
        // synchronous use of API requires we call ProcessResults now -- this will push results
        // to our callback registered above
        nErr = DNSServiceProcessResult( ServiceRef );
        if (nErr) {
        fprintf(stderr, "DNSServiceProcessResult returned %d\n", nErr );
        }
         */

            DNSServiceRefDeallocate( ServiceRef );
            return vRecords;
        }

        ///////////////////////////////////////////////////////////////////////
        /// Ask how to contact a particular service
        std::vector<ZeroConfURL> ResolveService(
                const std::string& sName,
                const std::string& sRegType,
                const std::string& sDomain = "local" ,
                uint32_t nFlags = 0,
                uint32_t nInterface = 0
                )
        {
            DNSServiceRef ServiceRef;
            m_bResolveComplete = false;
            DNSServiceResolve( &ServiceRef, nFlags, nInterface, sName.c_str(), sRegType.c_str(), sDomain.c_str(),
                    _ResolveReplyCallback, this );


            _HandleEvents( ServiceRef );

            DNSServiceRefDeallocate( ServiceRef );

            std::vector<ZeroConfURL> vRes = m_vResolvedURLS;
            m_vResolvedURLS.clear();
            return vRes;
        }

    private:

        ///////////////////////////////////////////////////////////////////////
        /// Callback to get the results of a ResolveService request.
        static void DNSSD_API _ResolveReplyCallback(
                DNSServiceRef client,
                DNSServiceFlags nFlags,
                uint32_t ifIndex,
                DNSServiceErrorType errorCode,
                const char *fullname,
                const char *hosttarget,
                uint16_t opaqueport,
                uint16_t txtLen,
                const unsigned char *txtRecord,
                void *pThis
                )
        {
            ((ZeroConf*)pThis)->ResolveReplyCallback(
                    client,
                    nFlags,
                    ifIndex,
                    errorCode,
                    fullname,
                    hosttarget,
                    opaqueport,
                    txtLen,
                    txtRecord
                    );
        }

        const char* _GetHostIP()
        {
            char buf[128];
            gethostname( buf,sizeof(buf) );
            struct hostent *hp = gethostbyname( buf );
            struct in_addr* in = (struct in_addr*)hp->h_addr;
            return inet_ntoa( *in );
        }

        void ResolveReplyCallback(
                DNSServiceRef ,//client,
                DNSServiceFlags nFlags,
                uint32_t ,//ifIndex,
                DNSServiceErrorType ,//errorCode,
                const char* ,//fullname,
                const char* hosttarget,
                uint16_t opaqueport,
                uint16_t ,//txtLen,
                const unsigned char * //txtRecord
                )
        {
            //const char *src = (char*)txtRecord;
            union { uint16_t s; u_char b[2]; }
            port = { opaqueport };
            uint16_t PortAsNumber = ((uint16_t)port.b[0]) << 8 | port.b[1];

            struct hostent *hp = gethostbyname( hosttarget );
/*
            // get the IP that matches the one returned by gethostname
            std::string sIP;
            unsigned int i=0;
            while( hp->h_addr_list[i] != NULL) {
                struct in_addr* in = (struct in_addr*)hp->h_addr_list[i];
                 sIP = _GetHostIP();
                if( sIP == std::string(inet_ntoa(*in))){
                    printf( "using option: %s\n", sIP.c_str() );
                }
                i++;
            }
*/
            struct in_addr* in = (struct in_addr*)hp->h_addr;
//            printf( "first option: %s\n", inet_ntoa(*in) );

            /*
            std::string sIP = inet_ntoa( *in );
            struct in_addr addr;
            inet_aton( sIP.c_str(), &addr );
            */
//            char buf[128];
//            gethostname( buf,sizeof(buf) );
//            printf("hosttarget = %s, %s\n", hosttarget, buf );

            ZeroConfURL url;
            url.m_sHost = inet_ntoa(*in);
            url.m_nPort = PortAsNumber;

            // add to list if not seen already
            bool bInList = false;
            for( size_t ii = 0; ii < m_vResolvedURLS.size(); ii++ ){
                ZeroConfURL& a = m_vResolvedURLS[ii];
                if( a.m_sHost == url.m_sHost && a.m_nPort == url.m_nPort ){
                    bInList = true;
                }
            }
            if( bInList == false ){
//                printf("%s can be reached at %s:%u\n", fullname, url.m_sHost.c_str(), PortAsNumber );
                m_vResolvedURLS.push_back(url);
            }


            if( !(nFlags & kDNSServiceFlagsMoreComing) ){
                m_bResolveComplete = true;
            }
        }


        ///////////////////////////////////////////////////////////////////////
        // Callback called with registration result
        static void DNSSD_API  _RegisterServiceCallback(
                DNSServiceRef /*sdRef*/,             //< Input:
                DNSServiceFlags,                 //< NA currently unused
                DNSServiceErrorType nErrorCode,  //< Input: kDNSServiceErr_NoError on success
                const char *sName,               //< Input: service name
                const char *sRegType,            //< Input: _tcp or _udp
                const char *sDomain,             //< Input: e.g. local
                void *pUserData                  //< Input: user supplied data
                )
        {
            ZeroConfRecord *pRecord = (ZeroConfRecord*)pUserData;
            if( nErrorCode != kDNSServiceErr_NoError ){
                printf("error in _RegisterServiceCallback\n");
            }
            else {
                pRecord->m_sServiceName = sName;
                pRecord->m_sRegType     = sRegType;
                pRecord->m_sDomain      = sDomain ? sDomain : "local";
            }
        }

        ///////////////////////////////////////////////////////////////////////
        ///
        static void DNSSD_API _BrowseReplyCallback(
                DNSServiceRef,
                DNSServiceFlags nFlags,
                uint32_t,
                DNSServiceErrorType nErrorCode,
                const char *sServiceName,
                const char * sRegType,
                const char * sReplyDomain,
                void *pUserData
                )
        {
            std::vector<ZeroConfRecord>& vRecords = *((std::vector<ZeroConfRecord>*)pUserData);

            if( nErrorCode != kDNSServiceErr_NoError ){
                printf("error _BrowseReplyCallback() -- %d\n", nErrorCode );
                return;
            }

//            static int ii = 0;
//            printf( "_BrowseReplyCallback called %d times\n", ii++ );

            /// kDNSServiceFlagsAdd  indicates that the service parameter contains the
            //  name of a service that has been found; you should add it to your list of
            //  available services.
            if( nFlags & kDNSServiceFlagsAdd ){
                ZeroConfRecord r;
                r.m_sServiceName = sServiceName;
                r.m_sRegType = sRegType;
                r.m_sDomain = sReplyDomain;
//                printf("adding %s.%s at %s\n", sServiceName, sRegType, sReplyDomain );
                // only record node if we haven't seen it yet -- else bail
                for( size_t ii = 0; ii < vRecords.size(); ii++ ){
                    if( vRecords[ii].m_sServiceName == sServiceName &&
                            vRecords[ii].m_sRegType == sRegType &&
                            vRecords[ii].m_sDomain == sReplyDomain ){
                        return;
                    }
                }

                vRecords.push_back( r );
            }

            /// done?
            if( !(nFlags & kDNSServiceFlagsMoreComing) ){
                //printf("no more coming\n");
            }
        }

        /*
        ///////////////////////////////////////////////////////////////////////
        static void DNSSD_API _TestResolveReplyCallback(
                DNSServiceRef client,
                DNSServiceFlags nFlags,
                uint32_t ifIndex,
                DNSServiceErrorType errorCode,
                const char *fullname,
                const char *hosttarget,
                uint16_t opaqueport,
                uint16_t txtLen,
                const unsigned char *txtRecord,
                void *pThis
                )
        {
            printf("_TestResolveReplyCallback()\n");
        }

        ///////////////////////////////////////////////////////////////////////
        static void DNSSD_API _TestBrowseReplyCallback(
                DNSServiceRef,
                DNSServiceFlags nFlags,
                uint32_t,
                DNSServiceErrorType nErrorCode,
                const char *sServiceName,
                const char * sRegType,
                const char * sReplyDomain,
                void *pUserData
                )
        {
            printf("TestBrowseReplyCallback\n");
        }

        ///////////////////////////////////////////////////////////////////////
        static void DNSSD_API  _TestRegisterServiceCallback(
                DNSServiceRef, //sdRef,             //< Input:
                DNSServiceFlags,                 //< NA currently unused
                DNSServiceErrorType nErrorCode,  //< Input: kDNSServiceErr_NoError on success
                const char *sName,               //< Input: service name
                const char *sRegType,            //< Input: _tcp or _udp
                const char *sDomain,             //< Input: e.g. local
                void *pUserData                  //< Input: user supplied data
                )
        {
            printf("_TestRegisterServiceCallback()\n");
        }
         */

        ///////////////////////////////////////////////////////////////////////
        /// sit on the DNSServiceRef file descriptor to wait for results from the server
        static void _HandleEvents2( void* pThis ) { ((ZeroConf*)pThis)->HandleEvents2(); }
        void HandleEvents2()
        {
            int dns_sd_fd = m_DNSServiceRef ? DNSServiceRefSockFD( m_DNSServiceRef ) : -1;
            int nfds = dns_sd_fd + 1;
            fd_set readfds;
            int result;
            bool bStopNow = false;

//            printf("_HandleEvents2 -- heard from server: ");

            while( !bStopNow )  {

                if( m_ListenToServerThread.interruption_requested() ){
                    printf("exiting thread\n");
                    return;
                }


                // 1. Set up the fd_set as usual here.
                // This example client has no file descriptors of its own,
                // but a real application would call FD_SET to add them to the set here
                FD_ZERO( &readfds );

                // 2. Add the fd for our client(s) to the fd_set
                if( m_DNSServiceRef ){
                    FD_SET( dns_sd_fd, &readfds );
                }

                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 50000;

                result = select( nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv );
                if( result > 0 ){
                    DNSServiceErrorType err = kDNSServiceErr_NoError;
                    if( m_DNSServiceRef && FD_ISSET(dns_sd_fd , &readfds)){
                        err = DNSServiceProcessResult( m_DNSServiceRef );
                    }
                    if (err) {
                        fprintf(stderr, "DNSServiceProcessResult returned %d\n", err);
                        bStopNow = true;
                    }
 //                   printf(" more...");
                }
                else if( result == 0 && m_bResolveComplete ){
//                    printf("done hearing from server\n");
//                    bStopNow = true;
                    //myTimerCallBack();
                }
                else{
//                    printf("select() returned %d errno %d %s\n", result, errno, strerror(errno) );
                    if (errno != EINTR){
//                        bStopNow = 1;
                    }
                    return;
                }
            }
        }


        ///////////////////////////////////////////////////////////////////////
        /// sit on the DNSServiceRef file descriptor to wait for results from the server
        void _HandleEvents( DNSServiceRef ServiceRef )
        {
            int dns_sd_fd = ServiceRef ? DNSServiceRefSockFD( ServiceRef ) : -1;
            int nfds = dns_sd_fd + 1;
            fd_set readfds;
            struct timeval tv;
            int result;
            bool bStopNow = false;

//            printf("ZeroConf::_HandleEvents()\n");

            while( !bStopNow )  {
                // 1. Set up the fd_set as usual here.
                // This example client has no file descriptors of its own,
                // but a real application would call FD_SET to add them to the set here
                FD_ZERO( &readfds );

                // 2. Add the fd for our client(s) to the fd_set
                if( ServiceRef ){
                    FD_SET( dns_sd_fd, &readfds );
                }

                // 3. Set up the timeout.
                tv.tv_sec = 0;
                tv.tv_usec = 10000;

                result = select( nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv );
                if( result > 0 ){
                    DNSServiceErrorType err = kDNSServiceErr_NoError;
                    if( ServiceRef && FD_ISSET(dns_sd_fd , &readfds)){
                        err = DNSServiceProcessResult( ServiceRef );
                    }
                    if (err) {
                        fprintf(stderr, "DNSServiceProcessResult returned %d\n", err);
                        bStopNow = true;
                    }
                }
                else if( result == 0 && m_bResolveComplete ){
                    bStopNow = true;
                    //myTimerCallBack();
                }
                else{
 //                   printf("select() returned %d errno %d %s\n", result, errno, strerror(errno) );
                    if (errno != EINTR){
                        bStopNow = 1;
                    }
                }
            }
        }

    private:
        DNSServiceRef             m_DNSServiceRef; // only use for our registered service
        std::vector<ZeroConfURL>  m_vResolvedURLS;
        bool                      m_bResolveComplete;
        ZeroConfRecord            m_Record;
        boost::thread             m_ListenToServerThread;
};

#endif


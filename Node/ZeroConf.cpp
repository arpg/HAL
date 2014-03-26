#include <Node/ZeroConf.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/types.h> // for u_char
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <miniglog/logging.h>

#ifdef HAVE_DNSSD
#include <dns_sd.h>
#endif  // HAVE_DNSSD

namespace hal {

ZeroConf::ZeroConf() : dns_service_ref_(0), stop_thread_(false) {}
ZeroConf::~ZeroConf() {
  stop_thread_ = true;
  listen_to_server_thread_.join();
}

inline void _BrowseReplyCallback(
    DNSServiceRef,
    DNSServiceFlags nFlags,
    uint32_t,
    DNSServiceErrorType nErrorCode,
    const char *sServiceName,
    const char * sRegType,
    const char * sReplyDomain,
    void *pUserData) {
#ifdef HAVE_DNSSD
  std::vector<ZeroConfRecord>& vRecords = *((std::vector<ZeroConfRecord>*)pUserData);

  if (nErrorCode != kDNSServiceErr_NoError) {
    LOG(ERROR) << "_BrowseReplyCallback() -- " << strerror(nErrorCode);
    return;
  }

  /// kDNSServiceFlagsAdd  indicates that the service parameter contains the
  //  name of a service that has been found; you should add it to your list of
  //  available services.
  if (nFlags & kDNSServiceFlagsAdd) {
    ZeroConfRecord r;
    r.service_name = sServiceName;
    r.reg_type = sRegType;
    r.domain = sReplyDomain;
    // only record node if we haven't seen it yet -- else bail
    for (size_t ii = 0; ii < vRecords.size(); ii++) {
      if (vRecords[ii].service_name == sServiceName &&
          vRecords[ii].reg_type == sRegType &&
          vRecords[ii].domain == sReplyDomain) {
        return;
      }
    }

    vRecords.push_back(r);
  }

  /// done?
  if (!(nFlags & kDNSServiceFlagsMoreComing)) {
  }
#endif  // HAVE_DNSSD
}

/// Callback to get the results of a ResolveService request.
inline void _ResolveReplyCallback(
    DNSServiceRef client,
    DNSServiceFlags nFlags,
    uint32_t ifIndex,
    DNSServiceErrorType errorCode,
    const char *fullname,
    const char *hosttarget,
    uint16_t opaqueport,
    uint16_t txtLen,
    const unsigned char *txtRecord,
    void *pThis) {
  ((ZeroConf*)pThis)->ResolveReplyCallback(client,
                                           nFlags,
                                           ifIndex,
                                           errorCode,
                                           fullname,
                                           hosttarget,
                                           opaqueport,
                                           txtLen,
                                           txtRecord);
}

inline void _RegisterServiceCallback(
    DNSServiceRef /*sdRef*/,             //< Input:
    DNSServiceFlags,                 //< NA currently unused
    DNSServiceErrorType nErrorCode,  //< Input: kDNSServiceErr_NoError on success
    const char *sName,               //< Input: service name
    const char *sRegType,            //< Input: _tcp or _udp
    const char *sDomain,             //< Input: e.g. local
    void *pUserData                  //< Input: user supplied data
                                     ) {
#ifdef HAVE_DNSSD
  ZeroConfRecord *pRecord = (ZeroConfRecord*)pUserData;
  if (nErrorCode != kDNSServiceErr_NoError) {
    LOG(ERROR) << "Error in _RegisterServiceCallback";
  } else {
    pRecord->service_name = sName;
    pRecord->reg_type     = sRegType;
    pRecord->domain      = sDomain ? sDomain : "local";
  }
#endif  // HAVE_DNSSD
}

bool ZeroConf::RegisterService(
    const std::string& sName, //<
    const std::string& sRegType, //<
    uint16_t nPort,
    const std::string& sDomain) {
#ifdef HAVE_DNSSD
  DNSServiceErrorType err = kDNSServiceErr_NameConflict;
  std::string sServiceName = sName;
  //int nServiceCount = 0;
  while (err == kDNSServiceErr_NameConflict) {
    err = DNSServiceRegister(
        &dns_service_ref_, // uninitialized DNSServiceRef, active till death
        0, // nFlags
        0, // interface index
        sServiceName.c_str(),
        sRegType.c_str(), // example "_ftp._tcp.". must be an
        //  underscore that is followed by 1 to 14 characters that
        // can be  letters, digits, or  hyphens.
        sDomain.c_str(), // null, or maybe "local"
        NULL, // host,  specifies the SRV target host name.
        // Most applications  do  not  spefiy this
        htons(nPort), // service port, in network byte order
        0, // txtLen, length of txtRecord
        NULL, // txtRecord, a properly  formatted  DNSTXT record
        _RegisterServiceCallback, // DNSServiceServiceRegisterReply
        &record_);

    // do renaming manually (since it doesn't work on linux)
    if (err == kDNSServiceErr_NameConflict) {
      return false;
    }
  }

  if (err != kDNSServiceErr_NoError) {
    LOG(ERROR) << "Error calling DNSServiceRegister() -- "  << err;
    return false;
  }

  // ok now setup a thread that watches the avahi socket
  listen_to_server_thread_ = std::thread(
      std::bind(&ZeroConf::HandleEvents2, this));

  return true;
#else
  return false;
#endif  // HAVE_DNSSD
}

std::vector<ZeroConfRecord> ZeroConf::BrowseForServiceType(
    const std::string& sServiceType,
    const std::string& sDomain) {
  std::vector<ZeroConfRecord> vRecords;
#ifdef HAVE_DNSSD
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
  if (nErr != kDNSServiceErr_NoError) {
    LOG(ERROR) <<"BrowseForServiceType() failed --" << nErr;
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
  FD_SET(dns_sd_fd, &readfds);

  // 3. Set up the timeout.
  tv.tv_sec = 1;
  tv.tv_usec = 0;

  while (true) {
    int result = select(nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv);
    if (result > 0) {
      DNSServiceErrorType err = kDNSServiceErr_NoError;
      if (ServiceRef && FD_ISSET(dns_sd_fd, &readfds)) {
        err = DNSServiceProcessResult(ServiceRef);
      }

      if (err != kDNSServiceErr_NoError) {
        LOG(ERROR) << "ZeroConf error: " << strerror(err);
        return vRecords;
      }
    } else if (result == 0)  {
      return vRecords;
    }
  }

  DNSServiceRefDeallocate(ServiceRef);
#endif  // HAVE_DNSSD
  return vRecords;
}

std::vector<ZeroConfURL> ZeroConf::ResolveService(
    const std::string& sName,
    const std::string& sRegType,
    const std::string& sDomain,
    uint32_t nFlags,
    uint32_t nInterface) {
#ifdef HAVE_DNSSD
  DNSServiceRef ServiceRef;
  resolve_complete_ = false;
  DNSServiceResolve(&ServiceRef, nFlags, nInterface, sName.c_str(),
                    sRegType.c_str(), sDomain.c_str(),
                    _ResolveReplyCallback, this);
  _HandleEvents(ServiceRef);
  DNSServiceRefDeallocate(ServiceRef);

  std::vector<ZeroConfURL> vRes = resolved_urls_;
  resolved_urls_.clear();
  return vRes;
#else
  return std::vector<ZeroConfURL>();
#endif  // HAVE_DNSSD
}

const char* ZeroConf::_GetHostIP() {
  char buf[128];
  gethostname(buf, sizeof(buf));
  struct hostent *hp = gethostbyname(buf);
  struct in_addr* in = (struct in_addr*)hp->h_addr;
  return inet_ntoa(*in);
}

void ZeroConf::ResolveReplyCallback(
    DNSServiceRef, //client,
    DNSServiceFlags nFlags,
    uint32_t, //ifIndex,
    DNSServiceErrorType, //errorCode,
    const char*, //fullname,
    const char* hosttarget,
    uint16_t opaqueport,
    uint16_t, //txtLen,
    const unsigned char * //txtRecord
                                    ) {
#ifdef HAVE_DNSSD
  //const char *src = (char*)txtRecord;
  union { uint16_t s; u_char b[2]; }
  port = { opaqueport };
  uint16_t PortAsNumber = ((uint16_t)port.b[0]) << 8 | port.b[1];

  struct hostent *hp = gethostbyname(hosttarget);
  struct in_addr* in = (struct in_addr*)hp->h_addr;

  ZeroConfURL url;
  url.host = inet_ntoa(*in);
  url.port = PortAsNumber;

  LG << "DNS resolved service URL " << url;

  // add to list if not seen already
  bool bInList = false;
  for (size_t ii = 0; ii < resolved_urls_.size(); ii++) {
    ZeroConfURL& a = resolved_urls_[ii];
    if (a.host == url.host && a.port == url.port) {
      bInList = true;
    }
  }
  if (!bInList) {
    resolved_urls_.push_back(url);
  }

  if (!(nFlags & kDNSServiceFlagsMoreComing)) {
    resolve_complete_ = true;
  }
#endif  // HAVE_DNSSD
}

void ZeroConf::HandleEvents2() {
#ifdef HAVE_DNSSD
  int dns_sd_fd =
      dns_service_ref_ ? DNSServiceRefSockFD(dns_service_ref_) : -1;
  int nfds = dns_sd_fd + 1;
  fd_set readfds;
  while (!stop_thread_) {
    // 1. Set up the fd_set as usual here.
    // This example client has no file descriptors of its own,
    // but a real application would call FD_SET to add them to the set here
    FD_ZERO(&readfds);

    // 2. Add the fd for our client(s) to the fd_set
    if (dns_service_ref_) {
      FD_SET(dns_sd_fd, &readfds);
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;

    int result = select(nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv);
    if (result > 0) {
      DNSServiceErrorType err = kDNSServiceErr_NoError;
      if (dns_service_ref_ && FD_ISSET(dns_sd_fd, &readfds)) {
        err = DNSServiceProcessResult(dns_service_ref_);
      }
      if (err) {
        LOG(ERROR) << "DNSServiceProcessResult returned " << err;
        stop_thread_ = true;
      }
    }
    else if (result == 0 && resolve_complete_) {
      // stop_thread_ = true;
    }
  }
#endif  // HAVE_DNSSD
}

void ZeroConf::_HandleEvents(DNSServiceRef ServiceRef) {
#ifdef HAVE_DNSSD
  int dns_sd_fd = ServiceRef ? DNSServiceRefSockFD(ServiceRef) : -1;
  int nfds = dns_sd_fd + 1;
  fd_set readfds;
  struct timeval tv;
  bool stop_thread_ = false;
  while (!stop_thread_) {
    // 1. Set up the fd_set as usual here.
    // This example client has no file descriptors of its own,
    // but a real application would call FD_SET to add them to the set here
    FD_ZERO(&readfds);

    // 2. Add the fd for our client(s) to the fd_set
    if (ServiceRef) {
      FD_SET(dns_sd_fd, &readfds);
    }

    // 3. Set up the timeout.
    tv.tv_sec = 0;
    tv.tv_usec = 10000;

    int result = select(nfds, &readfds, (fd_set*)NULL, (fd_set*)NULL, &tv);
    if (result > 0) {
      DNSServiceErrorType err = kDNSServiceErr_NoError;
      if (ServiceRef && FD_ISSET(dns_sd_fd, &readfds)) {
        err = DNSServiceProcessResult(ServiceRef);
      }
      if (err) {
        LOG(ERROR) << "DNSServiceProcessResult returned " << strerror(err);
        stop_thread_ = true;
      }
    } else if (result == 0 && resolve_complete_) {
      stop_thread_ = true;
    }
  }
#endif  // HAVE_DNSSD
}

}  // end namespace hal

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

struct ZeroConfRecord {
  std::string     service_name;    //  service name
  std::string     reg_type; // _tcp or _udp
  std::string     domain;  // e.g. local
};

struct ZeroConfURL {
  const char* Host() {
    return host.c_str();
  }

  uint32_t Port() {
    return port;
  }

  std::string host; // service host
  uint32_t port; // service port
};

/// This class provides a *sychronous* interface to the local mdns service using
// Avahi's cross-platform Bounjour compatability API.
class ZeroConf {
 public:
  ZeroConf();
  ~ZeroConf();

  /// Generic interface to just register a service and bail
  bool RegisterService(
      const std::string& sName, //<
      const std::string& sRegType, //<
      uint16_t nPort,
      const std::string& sDomain = "local");

  /// Browse for a particular service type, e.g.m "_ssh._tcp"
  std::vector<ZeroConfRecord> BrowseForServiceType(
      const std::string& sServiceType,
      const std::string& sDomain = "local");

  /// Ask how to contact a particular service
  std::vector<ZeroConfURL> ResolveService(
      const std::string& sName,
      const std::string& sRegType,
      const std::string& sDomain = "local" ,
      uint32_t nFlags = 0,
      uint32_t nInterface = 0);

  const char* _GetHostIP();

  void ResolveReplyCallback(
      DNSServiceRef , //client,
      DNSServiceFlags nFlags,
      uint32_t , //ifIndex,
      DNSServiceErrorType , //errorCode,
      const char* , //fullname,
      const char* hosttarget,
      uint16_t opaqueport,
      uint16_t , //txtLen,
      const unsigned char * //txtRecord
                            );
  void HandleEvents2();

  /// sit on the DNSServiceRef file descriptor to wait for results
  /// from the server
  void _HandleEvents(DNSServiceRef ServiceRef);

 private:
  DNSServiceRef dns_service_ref_; // only use for our registered service
  std::vector<ZeroConfURL>  resolved_urls_;
  bool resolve_complete_;
  ZeroConfRecord record_;
  boost::thread listen_to_server_thread_;
};

#endif

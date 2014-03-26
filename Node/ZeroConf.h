#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <Node/NodeConfig.h>

typedef struct _DNSServiceRef_t* DNSServiceRef;
typedef uint32_t DNSServiceFlags;
typedef int32_t DNSServiceErrorType;

namespace hal {

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
  virtual ~ZeroConf();

  ///
  /// Returns true if ZeroConf is connected to the DNS and Avahi system
  bool IsValid() {
#ifdef HAVE_DNSSD
    return true;
#else
    return false;
#endif
  }

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
 protected:
  const char* _GetHostIP();

 private:
  DNSServiceRef dns_service_ref_;
  std::vector<ZeroConfURL>  resolved_urls_;
  bool resolve_complete_;
  ZeroConfRecord record_;
  std::thread listen_to_server_thread_;
  bool stop_thread_;
};

}  // end namespace hal

// Stream operator overload for ZeroConfURL
namespace std {
inline ostream& operator<<(ostream& os, const hal::ZeroConfURL& u) {
  return os << u.host << ":" << u.port;
}

inline ostream& operator<<(ostream& os, const hal::ZeroConfRecord& r) {
  return os << "(" << r.reg_type << ":" << r.service_name
            << " @ " << r.domain << ")";
}
}

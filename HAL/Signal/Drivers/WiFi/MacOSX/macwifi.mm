#import <Foundation/Foundation.h>
#import <CoreWLAN/CoreWLAN.h>
#import <vector>
#import <string>

#include "../WiFiDriver.h"

std::vector<AccessPoint> scan(std::string ifname) {
  NSString *nsifname = [NSString stringWithCString:ifname.c_str()
    encoding:[NSString defaultCStringEncoding]];
  CWInterface* interface = [CWInterface interfaceWithName:nsifname];

  NSError* err = nil;
  NSArray* scanResult = [[interface scanForNetworksWithSSID:nil error:&err] allObjects];

  std::vector<AccessPoint> res;
  if (err) {
    NSLog(@"%@ (%ld)", [err localizedDescription], [err code]);
    return res;
  }

  for (CWNetwork* network in scanResult)
  {
    AccessPoint ap;
    ap.ssid = std::string([[network ssid] UTF8String]);
    ap.bssid = std::string([[network bssid] UTF8String]);
    ap.rssi = [network rssiValue];
    res.push_back(ap);
  }

  return res;
}

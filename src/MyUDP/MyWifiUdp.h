
#ifndef mywifiudp_h
#define mywifiudp_h

#include "MyMbedUdp.h"
#include <WiFi.h>

 
//namespace arduino {
class MyWifiUDP : public MyMbedUDP {
  NetworkInterface *getNetwork() {
    return WiFi.getNetwork();
  }
};

//}
#endif

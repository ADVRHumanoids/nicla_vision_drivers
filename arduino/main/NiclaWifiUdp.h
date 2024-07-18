
#ifndef NICLA_WIFI_UDP_H
#define NICLA_WIFI_UDP_H

#include "NiclaMbedUdp.h"
#include <WiFi.h>

 
//namespace arduino {
class NiclaWifiUDP : public NiclaMbedUDP {
  NetworkInterface *getNetwork() {
    return WiFi.getNetwork();
  }
};

//}
#endif //NICLA_WIFI_UDP_H

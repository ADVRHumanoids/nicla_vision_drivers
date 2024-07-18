 

#ifndef NICLA_MBED_SOCKET_CLASS_H
#define NICLA_MBED_SOCKET_CLASS_H

#include <Arduino.h>
#include "netsocket/NetworkInterface.h"

//namespace arduino {

class NiclaMbedSocketClass {

public:
  void config(IPAddress local_ip);

  /* Change Ip configuration settings disabling the dhcp client
        *
        * param local_ip: 	Static ip configuration as string
        */
  void config(const char* local_ip);

  /* Change Ip configuration settings disabling the dhcp client
        *
        * param local_ip: 	Static ip configuration
		* param dns_server:     IP configuration for DNS server 1
        */
  void config(IPAddress local_ip, IPAddress dns_server);

  /* Change Ip configuration settings disabling the dhcp client
        *
        * param local_ip: 	Static ip configuration
		* param dns_server:     IP configuration for DNS server 1
        * param gateway : 	Static gateway configuration
        */
  void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway);

  /* Change Ip configuration settings disabling the dhcp client
        *
        * param local_ip: 	Static ip configuration
		* param dns_server:     IP configuration for DNS server 1
        * param gateway: 	Static gateway configuration
        * param subnet:		Static Subnet mask
        */
  void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);

  /* Change DNS Ip configuration
     *
     * param dns_server1: ip configuration for DNS server 1
     */
  void setDNS(IPAddress dns_server1);

  /* Change DNS Ip configuration
     *
     * param dns_server1: ip configuration for DNS server 1
     * param dns_server2: ip configuration for DNS server 2
     *
     */
  void setDNS(IPAddress dns_server1, IPAddress dns_server2);

  /*
     * Get the interface IP address.
     *
     * return: Ip address value
     */
  IPAddress localIP();

  /*
     * Get the interface subnet mask address.
     *
     * return: subnet mask address value
     */
  IPAddress subnetMask();

  /*
     * Get the gateway ip address.
     *
     * return: gateway ip address value
     */
  IPAddress gatewayIP();

  /*
     * Get the DNS Server ip address.
     *
     * return: DNS Server ip address value
     */
  IPAddress dnsServerIP();

  /*
     * Get the DNS Server ip address.
     *
     * return: DNS Server ip address value
     */
  IPAddress dnsIP(int n = 0);

  virtual NetworkInterface* getNetwork() = 0;
  
  /*
     * Download a file from an HTTP endpoint and save it in the provided `target` location on the fs
     * The parameter cbk can be used to perform actions on the buffer before saving on the fs
     *
     * return: on success the size of the downloaded file, on error -status code
     */
  int download(
    const char* url, const char* target, bool const is_https = false);
  /*
     * Download a file from an HTTP endpoint and handle the body of the request on a callback
     * passed as an argument
     *
     * return: on success the size of the downloaded file, on error -status code
     */
  int download(
    const char* url, bool const is_https = false,
    mbed::Callback<void(const char*, uint32_t)> cbk = nullptr);

  int hostByName(const char* aHostname, IPAddress& aResult);

  uint8_t* macAddress(uint8_t* mac);
  String macAddress();

  void setFeedWatchdogFunc(voidFuncPtr func);
  void feedWatchdog();

  friend class NiclaMbedUDP;
  friend class MbedServer;
  friend class MbedClient;

protected:
  SocketAddress _ip = nullptr;
  SocketAddress _gateway = nullptr;
  SocketAddress _netmask = nullptr;
  SocketAddress _dnsServer1 = nullptr;
  SocketAddress _dnsServer2 = nullptr;
  bool _useStaticIP = false;

  voidFuncPtr _feed_watchdog_func = nullptr;

  FILE* download_target;

  void body_callback(const char* data, uint32_t data_len);

  static IPAddress ipAddressFromSocketAddress(SocketAddress socketAddress);
  static SocketAddress socketAddressFromIpAddress(IPAddress ip, uint16_t port);
  static nsapi_error_t gethostbyname(NetworkInterface* interface, const char* aHostname, SocketAddress* socketAddress);
};

using NiclaSocketHelpers = NiclaMbedSocketClass;

//}

#endif //NICLA_MBED_SOCKET_CLASS_H

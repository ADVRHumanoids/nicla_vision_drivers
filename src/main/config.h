#ifndef CONFIG_H
#define CONFIG_H
#define _UDP_ 0
#define _TCP_ 1

// User configurations:
#define _IP_ 10, 42, 0, 1 //192, 168, 1, 109 //10, 42, 0, 1     // your ROS-running machine's IP (example: 10.42.0.1)
#define NETWORK_SSID "DamianoHotspot"            // your network's ssid
#define NETWORK_KEY "DamianoHotspot"             // your network's password
#define NETWORK_TYPE _UDP_	                     // _TCP_ or _UDP_
#define ENABLE_ARDUINO_IDE_SERIAL_MONITOR true   // true or false
#define ENABLE_VERBOSE false                      // true or false

#endif //CONFIG_H

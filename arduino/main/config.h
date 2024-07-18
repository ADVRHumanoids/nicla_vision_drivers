#ifndef CONFIG_H
#define CONFIG_H
#define _UDP_ 0
#define _TCP_ 1

// User configurations:
#define _IP_ 10, 42, 0, 1 // your ROS-running machine's IP, with commas instead of dots (example for: 10.42.0.1)
#define NETWORK_SSID "your_ssid"            // your network's ssid
#define NETWORK_KEY "your_ssid_password"             // your network's password
#define NETWORK_TYPE _UDP_	                     // _TCP_ or _UDP_
#define ENABLE_ARDUINO_IDE_SERIAL_MONITOR false   // Must be false when using the board untethered to the Arduino IDE running PC
#define ENABLE_VERBOSE false                      // Some debug prints, printed only when using the board tethered to the Arduino IDE running PC
#define ENABLE_VERBOSE_TIME false                 // Some more debug prints regarding execution times, printed only when using the board tethered to the Arduino IDE running PC

#define USE_CAM true
#define USE_MIC true
#define USE_IMU true
#define USE_TOF true

#endif //CONFIG_H

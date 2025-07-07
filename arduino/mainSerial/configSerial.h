#ifndef CONFIG_H
#define CONFIG_H
// User configurations:
#define SERIAL_ENABLE_FOR_VERBOSE false //can not use this while communicating since everyhting use serial
#define ENABLE_VERBOSE false                      // Some debug prints, printed only when using the board tethered to the Arduino IDE running PC
                                                 // SERIAL_ENABLE_FOR_VERBOSE must be true to be used
#define ENABLE_VERBOSE_TIME false                 // Some more debug prints regarding execution times, printed only when using the board tethered to the Arduino IDE running PC

#define BAUDRATE 500000

#define USE_CAM true
#define USE_MIC false
#define USE_IMU false
#define USE_TOF false

#define COMPRESS_IMAGE true

#endif //CONFIG_H

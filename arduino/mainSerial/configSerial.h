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

//image options
#define CAM_PIXFORMAT CAMERA_RGB565 //CAMERA_BAYER or CAMERA_RGB565
#define CAM_COMPRESS false //jpeg compression, set to false when using CAMERA_BAYER
#define CAM_RES CAMERA_R320x240 //CAMERA_R160x120, CAMERA_R320x240, CAMERA_R320x320... CAMERA_R640x480 seems always wrong. USE CAMERA_R320x240 with CAMERA_RGB565. No memory for bigger res.
#define CAM_FPS 60 //30, 60

#endif //CONFIG_H

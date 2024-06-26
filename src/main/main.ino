#include "config.h"

#define SERIAL_ENABLE ENABLE_ARDUINO_IDE_SERIAL_MONITOR
#define VERBOSE SERIAL_ENABLE && ENABLE_VERBOSE
#define VERBOSE_TIME SERIAL_ENABLE && false  //internal debug for operation's time 

#if defined(NETWORK_TYPE) && NETWORK_TYPE == _UDP_
#include "MyWifiUdp.h"
#include <SPI.h>
#include <WiFi.h>
#include <Arduino.h> 
#define NETWORK_TYPE_STR "udp"
#else
#include <SPI.h>
#include <WiFi.h>
#include <Arduino.h> 
#include "MyWifiUdp.h"
#define NETWORK_TYPE_STR "tcp"
#endif


#include "gc2145.h" 
#include <PDM.h>
#include <memory>
#include <functional>
#include <JPEGENC.h>
#include "VL53L1X.h"

 
#define IMAGE_MODE CAMERA_RGB565

// // Microphone
// static const char channels = 1; // default number of output channels
// static const int frequency = 16000; // default PCM output frequency  
// short sampleBuffer[512]; // Buffer to read samples into, each sample is 16-bits
// volatile int samplesRead; // Number of audio samples read


// void onPDMdataProva() {
//   // Query the number of available bytes
//   int bytesAvailable = PDM.available(); 

//   // Read into the sample buffer
//   PDM.read(sampleBuffer, bytesAvailable);

//   // 16-bit, 2 bytes per sample
//   samplesRead = bytesAvailable / 2; 

// }

class StreamManager {
  public:
    StreamManager(const char* _ssid_, const char* _pwd_, 
                  IPAddress _remoteIP_, const char* _connection_type_, 
                  bool _verbose_);  // Constructor
 
    void blinkLED(const char * ledColor, uint32_t count);
    void switchOnLED(const char* ledColor);
    void switchOffLED(const char* ledColor);
    void connectWifi();
    void printWifiStatus();
    void startClientSocket();
    uint8_t testTCP();
    void initSensors();
    void onPDMdata();
    static void staticCallback(); 
    void sense_and_send();
    void run();
    uint16_t bytes_to_uint16(byte byte1, byte byte2);

 
    // # warning settings
    bool verbose;
    int error_timeout;

    // # error handling init
    bool error;
    int error_time;
    int error_network;
    int error_unforseen;
    int error_quality;

    // CONNECTION TYPE is
    //     # 1 for TCP, or
    //     # 0 for UDP
    uint8_t connection_type;

    // # wifi ssid and password
    const char* ssid;
    const char* password;

    // # server address and port
    IPAddress ip;
    unsigned int port = 8002; // remote port
    unsigned int localPort = 9001;

    // WiFi connection status
    int status = WL_IDLE_STATUS;

    // TCP Client
    WiFiClient client;
    WiFiClient test_connection_client;
    unsigned long start_attempt_test;
    // UDP Client 
    MyWifiUDP Udp;

    // # Define data types for headers (1 byte)
    const uint8_t IMAGE_TYPE = 0;     //0b00;
    const uint8_t AUDIO_TYPE = 1;     //0b01;
    const uint8_t DISTANCE_TYPE = 2;  //0b10;
    const uint8_t IMU_TYPE = 3;       //0b11;

    // # Defining package dimensions (bytes) utils
    const unsigned int int2bytesSize = 4; 
    const unsigned int sample_buff_size = sizeof(sampleBuffer[0]); 
    const unsigned int packetSize = 65000;
    const unsigned int headerLength = int2bytesSize + int2bytesSize + sizeof(IMAGE_TYPE);  // bytes (pkg size + timestamp size + data type size) = 4 + 4 + 1 =  9
    const unsigned int imuSize_ = headerLength - int2bytesSize + 6 * int2bytesSize;        // 6 Floats and each Float is 4 bytes (as the Int);
    const unsigned int distanceSize = headerLength - int2bytesSize + int2bytesSize;        // = 9 - 4 + 4 = 9   Note: this info is used in server -> we do -4 because pkg size is len(packet)-len(pkg size)) 
    const unsigned int audioSize = headerLength - int2bytesSize + sample_buff_size;        // = 9 - 4 + sample_buff_size = 1029 
    unsigned int imageSize = 0;                                                            // imageSize is variable, it depends on the compression


    unsigned long timestamp = 0;

    // Camera        
    GC2145 nicla_cam; 
    std::shared_ptr<Camera> camPtr;       
    int sample_rate = 60; 
    JPEGENC jpgenc;
    JPEGENCODE enc; 
    FrameBuffer fb; 
    int out_jpg_len;
    uint8_t im[320*120*3];
    // uint8_t fake_img[153600];
    

    // Microphone
    static const char channels = 1; // default number of output channels
    static const int frequency = 16000; // default PCM output frequency  
    static const short audio_buffer_size = 10; 
    volatile bool audio_buffer_filled = false;
    short sampleBuffer[10][512]; // Buffers to read samples into, each sample is 16-bits
    unsigned short audio_buffer_i = 0; // index for audio buffers
    volatile int samplesRead; // Number of audio samples read

    // ToF
    VL53L1X proximity;
    int reading = 0;

    //Debug - Verbose stuff
    unsigned long start_time;
    unsigned long end_time;
  
};


StreamManager::StreamManager(const char* _ssid_, const char* _pwd_, 
                            IPAddress _remoteIP_, const char* _connection_type_, 
                            bool _verbose_=false){
  
  ssid = _ssid_; // your network SSID (name)
  password = _pwd_; // your network password (use for WPA, or use as key for WEP) 
  ip = _remoteIP_; // IPAddress object with the desired IP address
  
  if (_connection_type_ == "tcp"){
    connection_type = 1;
  }
  else{
    connection_type = 0;
  }

  verbose = _verbose_;  

  // For debug:
  
  // for (int i = 0; i < 153600; i = i+2) {
  //   fake_img[i] = 6;
  //   fake_img[i+1] = 0;
  // }

  // for (int i = 0; i < 320*120*3; i = i+3) {
  //   im[i] = 255;
  //   im[i+1] = 0;
  //   im[i+2] = 0;
  // }

}

void StreamManager::blinkLED(const char* ledColor, uint32_t count = 0xFFFFFFFF){
  int LED = LED_BUILTIN;
  if (ledColor=="red"){
    LED = LEDR; 
  }
  else if (ledColor=="green"){
    LED = LEDG; 
  }
  else {
    LED = LEDB; 
  }

  pinMode(LED, OUTPUT);
  while (count--) {
    digitalWrite(LED, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(50);                       // wait for a second
    digitalWrite(LED, HIGH); // turn the LED off by making the voltage LOW
    delay(50);                       // wait for a second
  }
}

void StreamManager::switchOffLED(const char* ledColor){
  int LED = LED_BUILTIN;
  if (ledColor=="red"){
    LED = LEDR; 
  }
  else if (ledColor=="green"){
    LED = LEDG; 
  }
  else {
    LED = LEDB; 
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // turn the LED on (HIGH is the voltage level)
}

void StreamManager::switchOnLED(const char* ledColor){
  int LED = LED_BUILTIN;
  if (ledColor=="red"){
    LED = LEDR; 
  }
  else if (ledColor=="green"){
    LED = LEDG; 
  }
  else {
    LED = LEDB; 
  }

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); // turn the LED off (LOW is the voltage level)
}

void StreamManager::connectWifi(){

  this->switchOnLED("blue");

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    if(SERIAL_ENABLE) Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if(SERIAL_ENABLE) Serial.print("Attempting to connect to SSID: ");
    if(SERIAL_ENABLE) Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, password);
    delay(3000); // wait 3 seconds for connection:
  }

  if(SERIAL_ENABLE) Serial.println("Connected to wifi");
  this->printWifiStatus();

  this->switchOffLED("blue");
}

void StreamManager::printWifiStatus() {
  // print the SSID of the network you're attached to:
  if(SERIAL_ENABLE) Serial.print("SSID: ");
  if(SERIAL_ENABLE) Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  if(SERIAL_ENABLE) Serial.print("IP Address: ");
  if(SERIAL_ENABLE) Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  if(SERIAL_ENABLE) Serial.print("signal strength (RSSI):");
  if(SERIAL_ENABLE) Serial.print(rssi);
  if(SERIAL_ENABLE) Serial.println(" dBm");
}

void StreamManager::startClientSocket() {
  //     unsigned long startAttemptTime = millis();
  //     const unsigned long attemptDuration = 100000; // 10 seconds overall attempt duration
  //     const unsigned long retryInterval = 3000; // 3 seconds between retries
  //     while (millis() - startAttemptTime < attemptDuration)
  this->switchOnLED("blue");
  if (connection_type){
    if(SERIAL_ENABLE) Serial.println("\nStarting TCP socket connection to server...");

    start_time = micros();
    
    client.connect(ip, port); 

    end_time = micros();
    Serial.print("CONNECT TIME (us)"); Serial.println(end_time-start_time);

    while (!client.connected()) { 
      client.stop(); 
      client.connect(ip, port);  
      if(SERIAL_ENABLE) Serial.println("Not connected, retrying...");
      delay(1000); // wait 3 seconds for connection:
    }

    if(SERIAL_ENABLE) Serial.println("\nSocket connection to server established!");
  }
  else{
    if(SERIAL_ENABLE) Serial.println("\nStarting UDP socket connection to server...");
    Udp.begin(localPort);
  }

  start_attempt_test = millis();
  this->switchOffLED("blue");
}

uint8_t StreamManager::testTCP() {
  //     unsigned long startAttemptTime = millis();
  //     const unsigned long attemptDuration = 100000; // 10 seconds overall attempt duration
  //     const unsigned long retryInterval = 3000; // 3 seconds between retries
  //     while (millis() - startAttemptTime < attemptDuration)
   
  if(SERIAL_ENABLE) Serial.println("\nStarting test on TCP socket connection to server...");

  test_connection_client.connect(ip, port); 

  if (!test_connection_client.connected()) { 
    this->switchOnLED("blue");     
    if(SERIAL_ENABLE) Serial.println("Test not connected!"); 
    test_connection_client.stop();
    return 0; 
  }
  else{
    test_connection_client.stop();
    return 1; 
  }
}


/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void StreamManager::onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available(); 

  // Read into the sample buffer
  PDM.read(sampleBuffer[audio_buffer_i], bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2; //TODO is this still necessary?

  //TODO handle circular queue better, this assume it will never be filled
  if (audio_buffer_i < audio_buffer_size-1) {
    audio_buffer_i++;
  } else {
    audio_buffer_filled = true;
    audio_buffer_i = 0;
  }


}

/**************************************************************************************/
// Create StreamManager object
IPAddress remoteIP(_IP_);     
StreamManager stream_manager(NETWORK_SSID, NETWORK_KEY, remoteIP, NETWORK_TYPE_STR);
/**************************************************************************************/

// Static member function as a trampoline for the callback 
void StreamManager::staticCallback() {
    // Call the actual member function using the instance pointer
    stream_manager.onPDMdata();
}


void StreamManager::initSensors() {

  if(SERIAL_ENABLE) Serial.println("\nInitiating sensors...!\n");

  // Camera  
  camPtr = std::make_shared<Camera>(nicla_cam);
   
  // Init the cam QVGA, 30FPS 
  if (!camPtr->begin(CAMERA_R320x240, IMAGE_MODE, sample_rate)) {
    if(SERIAL_ENABLE) Serial.println("\nCamera not available!\n");
    this->switchOnLED("red");
  }

  if(SERIAL_ENABLE) Serial.println("\nCamera DONE...!\n");
 
  

  // ToF
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C
  proximity.setBus(&Wire1);

  if (!proximity.init()) {
    if(SERIAL_ENABLE) Serial.println("\nFailed to detect and initialize sensor!");
    this->switchOnLED("red");
  }

  proximity.setDistanceMode(VL53L1X::Long);
  proximity.setMeasurementTimingBudget(10000);
  proximity.startContinuous(10);

  if(SERIAL_ENABLE) Serial.println("\nToF DONE...!\n");



  // Microphone     
  PDM.onReceive(StreamManager::staticCallback);  // Configure the data receive callback
  // PDM.onReceive(onPDMdataProva);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    if(SERIAL_ENABLE) Serial.println("\nFailed to start PDM!");
    this->switchOnLED("red");
  }

  if(SERIAL_ENABLE) Serial.println("\nMic DONE...!\n");


  if(SERIAL_ENABLE) Serial.println("Init Sensors DONE !\n");
}


// Function to convert a pair of bytes to a 16-bit unsigned integer
uint16_t StreamManager::bytes_to_uint16(byte byte1, byte byte2) {
  return (byte1 << 8) | byte2;
}


void StreamManager::sense_and_send() {

  timestamp = millis();

  if (VERBOSE) {
      Serial.println("Sense and Send !");
      Serial.print("Timestamp: "); 
      Serial.println(timestamp);
  }

  // ToF
  if (VERBOSE_TIME){
    Serial.println("START LOOP");
    start_time = micros();
  } 

  reading = proximity.read();

  if (VERBOSE_TIME) {
    end_time = micros();
    Serial.print("TOF TIME GET (us)"); Serial.println(end_time-start_time);
  }
  if (VERBOSE){
    Serial.print("TOF reading (mm): ");
    Serial.println(reading);

    Serial.println("Preparing distance packet ... ");
  } 

  
  if (VERBOSE_TIME) start_time = micros();

  memcpy(im, &distanceSize, int2bytesSize );                                  // sizeof(distanceSize) = int2bytesSize
  memcpy(im+int2bytesSize, &timestamp, int2bytesSize );                       // sizeof(timestamp) = int2bytesSize
  memcpy(im+int2bytesSize*2, &DISTANCE_TYPE, sizeof(DISTANCE_TYPE) );         // sizeof(DISTANCE_TYPE) = 1
  memcpy(im+int2bytesSize*2+sizeof(DISTANCE_TYPE), &reading, int2bytesSize ); // sizeof(reading) = int2bytesSize
  
  if (VERBOSE_TIME) {
    end_time = micros();
    Serial.print("TOF TIME MEM (us)"); Serial.println(end_time-start_time);

    start_time = micros();
  }

  if (connection_type){
    client.write((uint8_t*)im, headerLength+int2bytesSize);
  }
  else{
    Udp.beginPacket(ip, port); 
    Udp.write((uint8_t*)im, headerLength+int2bytesSize);
    Udp.endPacket();
  }
    

  if (VERBOSE_TIME) {
    end_time = micros();
    Serial.print("TOF TIME SEND (us)"); Serial.println(end_time-start_time);
  }

  if (VERBOSE) Serial.println("Sent distance packet ! ");
     
  /***************************************/
  /***************************************/
 
  // Microphone
  // Wait for samples to be read 
  if (audio_buffer_filled) {
    if(SERIAL_ENABLE) Serial.println("AUDIO BUFFER HAS BEEN FILLED! BAD THINGS WILL HAPPEN WITH AUDIO\n");
    audio_buffer_filled = false;
  }

  if (audio_buffer_i > 0) {

    // DEBUG: Print samples to the serial monitor or plotter
    // for (int i = 0; i < samplesRead; i++) {
    //   Serial.println(sampleBuffer[i]);
    // }

       
    //TODO, can we send more mic packed all at once instead of doing this?
    for (unsigned short i = 0; i < audio_buffer_i; i++) {

      if (VERBOSE) Serial.println("Preparing audio packet ... ");

      if (VERBOSE_TIME) start_time = micros();

      memcpy(im, &audioSize, int2bytesSize );                                                     // sizeof(audioSize) = int2bytesSize
      memcpy(im+int2bytesSize, &timestamp, int2bytesSize );                                       // sizeof(timestamp) = int2bytesSize
      memcpy(im+int2bytesSize*2, &AUDIO_TYPE, sizeof(AUDIO_TYPE) );                               // sizeof(AUDIO_TYPE) = 1
      memcpy(im+int2bytesSize*2+sizeof(AUDIO_TYPE), (uint8_t*)sampleBuffer[i], sample_buff_size );   // Note: int2bytesSize*2+sizeof(AUDIO_TYPE) = headerLength
      
      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("MIC TIME MEM (us)"); Serial.println(end_time-start_time);
        start_time = micros();
      }

      if (connection_type){
        client.write((uint8_t*)im, headerLength+sample_buff_size); 
      }
      else{
        Udp.beginPacket(ip, port); 
        Udp.write((uint8_t*)im, headerLength+sample_buff_size);
        Udp.endPacket();
      }      

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("MIC TIME SEND (us)"); Serial.println(end_time-start_time);
      }

      if (VERBOSE) Serial.println("Sent audio packet ! ");
    }

    audio_buffer_i = 0; // Clear the audio buffer index 
    
    samplesRead = 0; // Clear the read count
  }

  /***************************************/
  /***************************************/

  // Camera  
  // // // // OPTION 1 : LOW QUALITY COMPRESSION // // // //

  // if (camPtr->grabFrame(fb, 500) == 0) {  

  //   if (VERBOSE_TIME) start_time = micros();

  //   uint8_t* out_jpg = fb.getBuffer();

  //   if (VERBOSE_TIME) {
  //     end_time = micros();
  //     Serial.print("IMG TIME GET (us)"); Serial.println(end_time-start_time);
  //   }

  //   int offset_out_jpg = 0; 

  //   for (int i = 0; i < 2; i++){  // Process the image at halves: 320 x 240 x 2  === (first half) 320 x 120 x 2 and (second half) 320 x 120 x 2
      
  //     // 1. Convert half image from RGB565 to RGB888
  //     if (VERBOSE) Serial.println("Converting!!!"); 
  //     if (VERBOSE_TIME) start_time = micros();

  //     int idx = 0;
  //     int start = 0;
  //     if (i){
  //       start = (320*240*2)-1; 
  //     }
  //     else{
  //       start = (320*240)-1; 
  //     }
  //     for (int j = start ; j > 320*240*i ; j -= 2 ){
  //       uint16_t px = this->bytes_to_uint16(out_jpg[j-1], out_jpg[j]);

  //       im[idx] = (px & 0xF800) >> 8;
  //       im[idx+1] = (px & 0x07E0) >> 3;
  //       im[idx+2] = (px & 0x001F) << 3;

  //       idx += 3;
  //     }

  //     if (VERBOSE_TIME) {
  //       end_time = micros();
  //       Serial.print("IMG TIME CONVERT (us)"); Serial.println(end_time-start_time);
  //     }

  //     // 2. Compress the half RGB888 image       
  //     if (VERBOSE) Serial.println("Encoding started...");        
  //     if (VERBOSE_TIME) start_time = micros();

  //     jpgenc.open(out_jpg+i*(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE))+int2bytesSize*2+sizeof(IMAGE_TYPE), 32768-int2bytesSize*2+sizeof(IMAGE_TYPE)+i*(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE))); 
  //     jpgenc.encodeBegin(&enc, 320, 120, JPEGE_PIXEL_RGB888, JPEGE_SUBSAMPLE_444, JPEGE_Q_LOW); 
  //     jpgenc.addFrame(&enc, im, 320 * 3); 
  //     out_jpg_len = jpgenc.close();
  //     if (!i){
  //       offset_out_jpg = out_jpg_len; 
  //     }

  //     if (VERBOSE_TIME) {
  //       end_time = micros();
  //       Serial.print("IMG TIME COMPRESS (us)"); Serial.println(end_time-start_time);
  //     }
  //     if (VERBOSE) {
  //       Serial.println("Encoding closed!");
  //       Serial.print("JPEG dimension (byte): "); 
  //       Serial.println(out_jpg_len);
  //     }
 
        
  //     // DEBUG:
  //     // memcpy(out_jpg, &out_jpg_len, int2bytesSize); // Copy the bytes of the dimension number into the array's head
      
      
  //     // 3. Send it through TCP
  //     // DEBUG:
  //     // client.write(out_jpg, out_jpg_len+sizeof(out_jpg_len));

  //     if (VERBOSE) Serial.println("Preparing image packet ... ");
  //     if (VERBOSE_TIME) start_time = micros();

  //     imageSize = headerLength - int2bytesSize + out_jpg_len; 

  //     memcpy(out_jpg+i*(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE)), &imageSize, int2bytesSize );                        // sizeof(imageSize) = int2bytesSize
  //     memcpy(out_jpg+i*(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE))+int2bytesSize, &timestamp, int2bytesSize );          // sizeof(timestamp) = int2bytesSize
  //     memcpy(out_jpg+i*(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE))+int2bytesSize*2, &IMAGE_TYPE, sizeof(IMAGE_TYPE) );  // sizeof(IMAGE_TYPE) = 1
      
      
  //   }   // end camera proc loop

  //   if (VERBOSE_TIME) {
  //     end_time = micros();
  //     Serial.print("IMG TIME MEM (us)"); Serial.println(end_time-start_time);
  //     start_time = micros();
  //   }

  //   if (connection_type){
  //     client.write(out_jpg, headerLength+out_jpg_len+headerLength+offset_out_jpg);
  //   }
  //   else{
  //     Udp.beginPacket(ip, port); 
  //     Udp.write(out_jpg, headerLength+out_jpg_len+headerLength+offset_out_jpg);
  //     Udp.endPacket();
  //   }

  //   if (VERBOSE_TIME) {
  //     end_time = micros();
  //     Serial.print("IMG TIME SEND (us)"); Serial.println(end_time-start_time);
  //   }

  //   if (VERBOSE) Serial.println("Sent image packet ! ");

  // }  // end camera if
  
  ///////////////////////////////////////////////////////////// 
  // // // // OPTION 2: MEDIUM QUALITY COMPRESSION // // // //
  if (camPtr->grabFrame(fb, 500) == 0) {  

    if (VERBOSE_TIME) start_time = micros();

    uint8_t* out_jpg = fb.getBuffer();

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME GET (us)"); Serial.println(end_time-start_time);
    }

    int offset_out_jpg = 0; 

    for (int i = 0; i < 2; i++){  // Process the image at halves: 320 x 240 x 2  === (first half) 320 x 120 x 2 and (second half) 320 x 120 x 2
      
      // 1. Convert half image from RGB565 to RGB888
      if (VERBOSE) Serial.println("Converting!!!"); 
      if (VERBOSE_TIME) start_time = micros();

      int idx = 0;
      int start = 0;
      if (i){
        start = (320*240*2)-1; 
      }
      else{
        start = (320*240)-1; 
      }
      for (int j = start ; j > 320*240*i ; j -= 2 ){
        uint16_t px = this->bytes_to_uint16(out_jpg[j-1], out_jpg[j]);

        im[idx] = (px & 0xF800) >> 8;
        im[idx+1] = (px & 0x07E0) >> 3;
        im[idx+2] = (px & 0x001F) << 3;

        idx += 3;
      }

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("IMG TIME CONVERT (us)"); Serial.println(end_time-start_time);
      }

      // 2. Compress the half RGB888 image       
      if (VERBOSE) Serial.println("Encoding started...");        
      if (VERBOSE_TIME) start_time = micros();

      jpgenc.open(out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE)+sizeof(IMAGE_TYPE), 32768-int2bytesSize*2+sizeof(IMAGE_TYPE)+sizeof(IMAGE_TYPE)); ////
      jpgenc.encodeBegin(&enc, 320, 120, JPEGE_PIXEL_RGB888, JPEGE_SUBSAMPLE_444, JPEGE_Q_MED); 
      jpgenc.addFrame(&enc, im, 320 * 3); 
      out_jpg_len = jpgenc.close();
      if (!i){
        offset_out_jpg = out_jpg_len; 
      }

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("IMG TIME COMPRESS (us)"); Serial.println(end_time-start_time);
      }
      if (VERBOSE) {
        Serial.println("Encoding closed!");
        Serial.print("JPEG dimension (byte): "); 
        Serial.println(out_jpg_len);
      }
        
      // DEBUG:
      // memcpy(out_jpg, &out_jpg_len, int2bytesSize); // Copy the bytes of the dimension number into the array's head
      
      
      // 3. Send it through TCP
      // DEBUG:
      // client.write(out_jpg, out_jpg_len+sizeof(out_jpg_len));

      if (VERBOSE) Serial.println("Preparing image packet ... ");
      if (VERBOSE_TIME) start_time = micros();

      imageSize = headerLength - int2bytesSize + out_jpg_len + sizeof(IMAGE_TYPE);   ////

      memcpy(out_jpg, &imageSize, int2bytesSize );                        // sizeof(imageSize) = int2bytesSize
      memcpy(out_jpg+int2bytesSize, &timestamp, int2bytesSize );          // sizeof(timestamp) = int2bytesSize
      memcpy(out_jpg+int2bytesSize*2, &IMAGE_TYPE, sizeof(IMAGE_TYPE) );  // sizeof(IMAGE_TYPE) = 1

      if(!i){
        memcpy(out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE), &IMAGE_TYPE, sizeof(IMAGE_TYPE));  ////
      }
      else{
        memcpy(out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE), &AUDIO_TYPE, sizeof(AUDIO_TYPE));  ////
      }


      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("IMG TIME MEM (us)"); Serial.println(end_time-start_time);
        start_time = micros();
      }

      if (connection_type){
        client.write(out_jpg, headerLength+out_jpg_len);
      }
      else{
        Udp.beginPacket(ip, port); 
        Udp.write(out_jpg, headerLength+out_jpg_len+sizeof(IMAGE_TYPE)); ////
        // Udp.write(out_jpg+(offset_out_jpg+int2bytesSize*2+sizeof(IMAGE_TYPE)), headerLength+out_jpg_len);
        Udp.endPacket();
      }

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("IMG TIME SEND (us)"); Serial.println(end_time-start_time);
      }

      if (VERBOSE) Serial.println("Sent image packet ! ");
      
    }   // end camera proc loop

  }  // end camera if

  if (VERBOSE_TIME) {
    Serial.println("END LOOP");
    Serial.println("");
  }

}


void StreamManager::run() {
 
  if (WiFi.status() != WL_CONNECTED){
    this->connectWifi();
  }

  if (connection_type){
    if (!client.connected()) { 
      this->switchOnLED("blue");
      if (VERBOSE) Serial.println();
      if (VERBOSE)  Serial.println("Server disconnected! Trying to establish new connection...");
      client.stop();
      this->startClientSocket();    
    }
    else if (millis()-start_attempt_test >= 60000){
      if(!this->testTCP()){
        this->switchOnLED("blue");
        // if (VERBOSE) Serial.println();
        // if (VERBOSE)  
        Serial.println("Server disconnected caught by test! Trying to establish new connection...");
        client.stop();
        this->startClientSocket(); 
      }
      else{
        start_attempt_test = millis();
      }
    }
  }

  this->sense_and_send();
}


void setup() {
  // put your setup code here, to run once:

  // Initialize serial and wait for port to open:
  if(SERIAL_ENABLE) {
    Serial.begin(500000); //9600 921600
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
  }
 
  // DEBUG SIZE 
  // Serial.print("SIZE OF INT: ");
  // Serial.println(sizeof(int));

  // Serial.print("SIZE OF SHORT: ");
  // Serial.println(sizeof(short));

  // Serial.print("SIZE OF MILLIS (TIME): ");
  // Serial.print(sizeof(unsigned long));
  // Serial.print(" MILLIS: ");
  // Serial.println(millis());

  // Serial.print("SIZE OF AUDIO BUF: ");
  // Serial.println(sizeof(stream_manager.sampleBuffer)); 
  // //

  stream_manager.blinkLED("green", 3);

  stream_manager.connectWifi(); 
 
  stream_manager.blinkLED("blue", 3);
 
  stream_manager.startClientSocket(); 
 
  stream_manager.blinkLED("green", 3);

  stream_manager.initSensors();




  stream_manager.switchOnLED("green");

  delay(3000);

  stream_manager.switchOffLED("green");  
}

void loop() {
  // put your main code here, to run repeatedly:

    

  stream_manager.run(); 
}

#include "configSerial.h"

#define VERBOSE SERIAL_ENABLE_FOR_VERBOSE && ENABLE_VERBOSE
#define VERBOSE_TIME SERIAL_ENABLE_FOR_VERBOSE && ENABLE_VERBOSE_TIME

#include <SPI.h>
#include <Arduino.h> 

#include "SerialTransfer.h"

#include <memory>
#include <functional>

#if USE_CAM
  #include "gc2145.h"
  #include <CRC32.h>
  #if COMPRESS_IMAGE
    #include <JPEGENC.h>
  #endif
#endif
#if USE_MIC 
  #include <PDM.h>
#endif
#if USE_TOF
  #include "VL53L1X.h"
#endif
#if USE_IMU
  #include <Arduino_LSM6DSOX.h>
#endif



class StreamManager {
  public:
    StreamManager(bool _verbose_);  // Constructor
 
    void blinkLED(const char * ledColor, uint32_t count);
    void switchOnLED(const char* ledColor);
    void switchOffLED(const char* ledColor);
    void initSensors();
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


    // # Define data types for headers (1 byte)
    const uint8_t IMAGE_TYPE = 0;     //0b00;
    const uint8_t AUDIO_TYPE = 1;     //0b01;
    const uint8_t DISTANCE_TYPE = 2;  //0b10;
    const uint8_t IMU_TYPE = 3;       //0b11;

    const uint8_t START_BYTE = 255; //0b11111111;
    const uint8_t END_BYTE = 254; //0b11111110;

    // # Defining package dimensions (bytes) utils
    const unsigned int int2bytesSize = 4; 
    const unsigned int headerLength = int2bytesSize + int2bytesSize + sizeof(IMAGE_TYPE);  // bytes (pkg size + timestamp size + data type size) = 4 + 4 + 1 =  9
    const unsigned int imuSize = headerLength - int2bytesSize + 6 * int2bytesSize;        // 6 Floats and each Float is 4 bytes (as the Int);
    const unsigned int distanceSize = headerLength - int2bytesSize + int2bytesSize;        // = 9 - 4 + 4 = 9   Note: this info is used in server -> we do -4 because pkg size is len(packet)-len(pkg size)) 
    #if USE_MIC
      const unsigned int sample_buff_size = sizeof(sampleBuffer[0]); 
      const unsigned int audioSize = headerLength - int2bytesSize + sample_buff_size;        // = 9 - 4 + sample_buff_size = 1029 
    #endif

    unsigned int imageSize = 0;                                                            // imageSize is variable, it depends on the compression


    unsigned long timestamp = 0;
    //generic data buffer for multiple sensors. Size is the max data (images). 
    #if USE_CAM && CAM_COMPRESS  
        uint8_t data_buffer[320*240*3]; 
    #elif USE_MIC
      uint8_t data_buffer[1+9+512*2+10+1]; //start_byte + headerLength + 512*short + un qualcosa nonsisamai + end_byte
    #elif USE_IMU
      uint8_t data_buffer[1+9+6*4+10+1]; //start_byte +headerLength + 6*float + un qualcosa nonsisamai + end_byte
    #else
      //tof just an int, cam without compression use the framebuffer and data_buffer just for the header
      //even if only cam and no compress would require smaller buffer, lets use this size anyway
      uint8_t data_buffer[1+9+4+10+1]; //start_byte + headerLength + int + un qualcosa nonsisamai + end_byte
    #endif

    // Camera  
    #if USE_CAM      
      GC2145 nicla_cam; 
      std::shared_ptr<Camera> camPtr;       
      FrameBuffer fb;
      int img_ptr_len;
      // uint8_t fake_img[153600];
      #if CAM_COMPRESS
        JPEGENC jpgenc;
        JPEGENCODE enc; 
      #endif
    #endif


    // Microphone
    #if USE_MIC      
      static const char channels = 1; // default number of output channels
      static const int frequency = 16000; // default PCM output frequency  
      static const short audio_buffer_size = 10; 
      short sampleBuffer[10][512]; // Buffers to read samples into, each sample is 16-bits
      volatile bool audio_buffer_filled = false;
      unsigned short audio_buffer_i = 0; // index for audio buffers
      volatile int samplesRead; // Number of audio samples read
      void onPDMdata();
      static void staticCallback(); 
    #endif

    // ToF
    #if USE_TOF
      VL53L1X proximity;
      int reading = 0;
    #endif

    // IMU
    #if USE_IMU
      float acc_x = 0;
      float acc_y = 0;
      float acc_z = 0;
      float gyro_x = 0;
      float gyro_y = 0; 
      float gyro_z = 0;
    #endif

    //Debug - Verbose stuff
    unsigned long start_time;
    unsigned long end_time;
  
};


StreamManager::StreamManager(bool _verbose_=false){
  
  verbose = _verbose_;  

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

#if USE_MIC
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
#endif

/**************************************************************************************/
// Create StreamManager object
StreamManager stream_manager;

/**************************************************************************************/

#if USE_MIC
// Static member function as a trampoline for the callback 
void StreamManager::staticCallback() {
    // Call the actual member function using the instance pointer
    stream_manager.onPDMdata();
}
#endif


void StreamManager::initSensors() {

  if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nInitiating sensors...!\n");


  #if USE_IMU
    if (!IMU.begin()) {
      if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nFailed to initialize IMU!");
      this->switchOnLED("red");
    }
    if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nIMU DONE...!\n");
  #endif


  #if USE_CAM      
    camPtr = std::make_shared<Camera>(nicla_cam);
    
    // Init the cam QVGA, 30FPS 
    if (!camPtr->begin(CAM_RES, CAM_PIXFORMAT, CAM_FPS)) {
      if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nCamera not available!\n");
      this->switchOnLED("red");
    }

    if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nCamera DONE...!\n");
  #endif
 

  #if USE_TOF
    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C
    proximity.setBus(&Wire1);

    if (!proximity.init()) {
      if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nFailed to detect and initialize sensor!");
      this->switchOnLED("red");
    }

    proximity.setDistanceMode(VL53L1X::Medium);
    proximity.setMeasurementTimingBudget(50000); //in microseconds
    proximity.startContinuous(50); //comment is readsingle() is used

    if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nToF DONE...!\n");
  #endif


  #if USE_MIC    
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
      if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nFailed to start PDM!");
      this->switchOnLED("red");
    }
    if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("\nMic DONE...!\n");
  #endif


  if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("Init Sensors DONE !\n");
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

  #if USE_TOF
  if (VERBOSE_TIME){
    Serial.println("START LOOP");
    start_time = micros();
  } 

  reading = proximity.read(); //default blocking
  //reading = proximity.readSingle(); //default blocking

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


  memcpy(data_buffer, &START_BYTE, sizeof(START_BYTE));
  memcpy(data_buffer+sizeof(START_BYTE), &distanceSize, int2bytesSize );                                  // sizeof(distanceSize) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize, &timestamp, int2bytesSize );                       // sizeof(timestamp) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2, &DISTANCE_TYPE, sizeof(DISTANCE_TYPE) );         // sizeof(DISTANCE_TYPE) = 1
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(DISTANCE_TYPE), &reading, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(DISTANCE_TYPE)+int2bytesSize, &END_BYTE, sizeof(END_BYTE) );

  if (VERBOSE_TIME) {
    end_time = micros();
    Serial.print("TOF TIME MEM (us)"); Serial.println(end_time-start_time);

    start_time = micros();
  }

  //Serial (USB) way of sending data
  Serial.write((uint8_t*)data_buffer, sizeof(START_BYTE)+headerLength+int2bytesSize+sizeof(END_BYTE));
    
  if (VERBOSE_TIME) {
    end_time = micros();
    Serial.print("TOF TIME SEND (us)"); Serial.println(end_time-start_time);
  }

  if (VERBOSE) Serial.println("Sent distance packet ! ");

  #endif //USE_TOF
  /***************************************/

  #if USE_IMU

  if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(acc_x, acc_y, acc_z);
  } 

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }

  if (VERBOSE){
    Serial.print("IMU reading: "); 
    Serial.print(acc_x); Serial.print('\t'); Serial.print(acc_y); Serial.print('\t'); Serial.println(acc_z); Serial.println();
    Serial.print(gyro_x); Serial.print('\t'); Serial.print(gyro_y); Serial.print('\t'); Serial.println(gyro_z); Serial.println();
    Serial.println("Preparing IMU packet ... ");
  } 

  memcpy(data_buffer, &START_BYTE, sizeof(START_BYTE));
  memcpy(data_buffer+sizeof(START_BYTE), &imuSize, int2bytesSize );                                  // sizeof(imuSize) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize, &timestamp, int2bytesSize );                       // sizeof(timestamp) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2, &IMU_TYPE, sizeof(IMU_TYPE) );         // sizeof(IMU_TYPE) = 1
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE), &acc_x, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+int2bytesSize, &acc_y, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+(2*int2bytesSize), &acc_z, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+(3*int2bytesSize), &gyro_x, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+(4*int2bytesSize), &gyro_y, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+(5*int2bytesSize), &gyro_z, int2bytesSize ); // sizeof(reading) = int2bytesSize
  memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMU_TYPE)+(6*int2bytesSize), &END_BYTE, sizeof(END_BYTE) ); 

  //Serial way of sending data
  Serial.write((uint8_t*)data_buffer, sizeof(START_BYTE)+headerLength+6*int2bytesSize+sizeof(END_BYTE));

  if (VERBOSE) Serial.println("Sent IMU packet ! ");
  #endif //USE_IMU

  /***************************************/
 
  #if USE_MIC
  // Wait for samples to be read 
  if (audio_buffer_filled) {
    if(SERIAL_ENABLE_FOR_VERBOSE) Serial.println("AUDIO BUFFER HAS BEEN FILLED! BAD THINGS WILL HAPPEN WITH AUDIO\n");
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

      memcpy(data_buffer, &START_BYTE, sizeof(START_BYTE) );                                                  
      memcpy(data_buffer+sizeof(START_BYTE), &audioSize, int2bytesSize );                                                     // sizeof(audioSize) = int2bytesSize
      memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize, &timestamp, int2bytesSize );                                       // sizeof(timestamp) = int2bytesSize
      memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2, &AUDIO_TYPE, sizeof(AUDIO_TYPE) );                               // sizeof(AUDIO_TYPE) = 1
      memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(AUDIO_TYPE), (uint8_t*)sampleBuffer[i], sample_buff_size );   // Note: int2bytesSize*2+sizeof(AUDIO_TYPE) = headerLength
      memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(AUDIO_TYPE)+sample_buff_size, &END_BYTE, sizeof(END_BYTE) );     

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("MIC TIME MEM (us)"); Serial.println(end_time-start_time);
        start_time = micros();
      }
 
      //Serial way of sending data
      Serial.write((uint8_t*)data_buffer, sizeof(START_BYTE)+headerLength+sample_buff_size+sizeof(END_BYTE));   

      if (VERBOSE_TIME) {
        end_time = micros();
        Serial.print("MIC TIME SEND (us)"); Serial.println(end_time-start_time);
      }

      if (VERBOSE) Serial.println("Sent audio packet ! ");
    }

    audio_buffer_i = 0; // Clear the audio buffer index 
    
    samplesRead = 0; // Clear the read count
  }
  #endif //USE_MIC
  /***************************************/

  #if USE_CAM    

  #if CAM_COMPRESS 

  if (camPtr->grabFrame(fb, 500) == 0) {  

    if (VERBOSE_TIME) start_time = micros();

    uint8_t* img_ptr = fb.getBuffer();

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME GET (us)"); Serial.println(end_time-start_time);
    }
      
    // 1. Convert half image from RGB565 to RGB888
    if (VERBOSE) Serial.println("Converting!!!"); 
    if (VERBOSE_TIME) start_time = micros();

    int idx = 0;
    int start = (320*240*2)-1;

    for (int j = start ; j > 0 ; j -= 2 ){
      uint16_t px = this->bytes_to_uint16(img_ptr[j-1], img_ptr[j]);

      //converting a 16-bit RGB565 pixel into 24-bit RGB888 format
      data_buffer[idx] = (px & 0xF800) >> 8;
      data_buffer[idx+1] = (px & 0x07E0) >> 3;
      data_buffer[idx+2] = (px & 0x001F) << 3;

      idx += 3;
    }

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME CONVERT (us)"); Serial.println(end_time-start_time);
    }

    // 2. Compress the RGB888 image       
    if (VERBOSE) Serial.println("Encoding started...");        
    if (VERBOSE_TIME) start_time = micros();

    //reserve space at the beginning for the header
    jpgenc.open(img_ptr+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMAGE_TYPE), 32768+sizeof(START_BYTE)-int2bytesSize*2+sizeof(IMAGE_TYPE)+sizeof(END_BYTE)); ////
    jpgenc.encodeBegin(&enc, 320, 240, JPEGE_PIXEL_RGB888, JPEGE_SUBSAMPLE_444, JPEGE_Q_MED); 
    //compress data_buffer image (not compressed, but converted to RGB888) into img_ptr, overwriting original image which is not necessary anymore
    jpgenc.addFrame(&enc, data_buffer, 320 * 3); 
    img_ptr_len = jpgenc.close();

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME COMPRESS (us)"); Serial.println(end_time-start_time);
    }
    if (VERBOSE) {
      Serial.println("Encoding closed!");
      Serial.print("JPEG dimension (byte): "); 
      Serial.println(img_ptr_len);
    }
      
    // DEBUG:
    // memcpy(img_ptr, &img_ptr_len, int2bytesSize); // Copy the bytes of the dimension number into the array's head
    
    
    // 3. Send it through Serial
    // DEBUG:
    // client.write(img_ptr, img_ptr_len+sizeof(img_ptr_len));

    if (VERBOSE) Serial.println("Preparing image packet ... ");
    if (VERBOSE_TIME) start_time = micros();

    imageSize = headerLength - int2bytesSize + img_ptr_len;   ////

    //crc-32
    CRC32 crc;
    crc.update(img_ptr+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMAGE_TYPE), img_ptr_len);
    timestamp = crc.finalize();

    memcpy(img_ptr, &START_BYTE, sizeof(START_BYTE));                      
    memcpy(img_ptr+sizeof(START_BYTE), &imageSize, int2bytesSize );                        // sizeof(imageSize) = int2bytesSize
    memcpy(img_ptr+sizeof(START_BYTE)+int2bytesSize, &timestamp, int2bytesSize );          // sizeof(timestamp) = int2bytesSize
    memcpy(img_ptr+sizeof(START_BYTE)+int2bytesSize*2, &IMAGE_TYPE, sizeof(IMAGE_TYPE) );  // sizeof(IMAGE_TYPE) = 1
    memcpy(img_ptr+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMAGE_TYPE)+img_ptr_len, &END_BYTE, sizeof(END_BYTE));  ////


    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME MEM (us)"); Serial.println(end_time-start_time);
      start_time = micros();
    }

    //Serial way of sending
    Serial.write(img_ptr, sizeof(START_BYTE)+headerLength+img_ptr_len+sizeof(END_BYTE));

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME SEND (us)"); Serial.println(end_time-start_time);
    }

    if (VERBOSE) Serial.println("Sent image packet ! ");
    

  } 

  #else //not CAM_COMPRESS
  if (camPtr->grabFrame(fb, 500) == 0) {  

    if (VERBOSE_TIME) start_time = micros();

    uint8_t* img_ptr = fb.getBuffer();

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME GET (us)"); Serial.println(end_time-start_time);
    }

    if (VERBOSE) Serial.println("Preparing image packet ... ");
    if (VERBOSE_TIME) start_time = micros();

    unsigned int imagePacketSize = headerLength - int2bytesSize + camPtr->frameSize();   ////

    //TEST using timestamp bytes for checksum instead

    //checksum
    // timestamp = 0;
    // for (int i = 0; i < camPtr->frameSize(); i++) {
    //     timestamp += img_ptr[i];
    // }

    //crc-32
    CRC32 crc;
    crc.update(img_ptr, camPtr->frameSize());
    timestamp = crc.finalize();

    memcpy(data_buffer, &START_BYTE, sizeof(START_BYTE));                      
    memcpy(data_buffer+sizeof(START_BYTE), &imagePacketSize, int2bytesSize );                        // sizeof(imageSize) = int2bytesSize
    memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize, &timestamp, int2bytesSize );          // sizeof(timestamp) = int2bytesSize
    memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2, &IMAGE_TYPE, sizeof(IMAGE_TYPE) );  // sizeof(IMAGE_TYPE) = 1

    //memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMAGE_TYPE), (uint8_t*)img_ptr, camPtr->frameSize());
    //memcpy(data_buffer+sizeof(START_BYTE)+int2bytesSize*2+sizeof(IMAGE_TYPE)+camPtr->frameSize(), &END_BYTE, sizeof(END_BYTE));  ////

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME MEM (us)"); Serial.println(end_time-start_time);
      start_time = micros();
    }

    //Serial way of sending
    //Serial.write((uint8_t*)data_buffer, sizeof(START_BYTE)+headerLength+camPtr->frameSize()+sizeof(END_BYTE));

    Serial.write((uint8_t*)data_buffer, sizeof(START_BYTE)+headerLength);
    Serial.write((uint8_t*)img_ptr, camPtr->frameSize());
    Serial.write(END_BYTE);

    if (VERBOSE_TIME) {
      end_time = micros();
      Serial.print("IMG TIME SEND (us)"); Serial.println(end_time-start_time);
    }

    if (VERBOSE) Serial.println("Sent image packet ! ");
    

  }  

  #endif //CAM_COMPRESS 
  #endif //USE_CAM

  /***************************************/


  if (VERBOSE_TIME) {
    Serial.println("END LOOP");
    Serial.println("");
  }

}


void StreamManager::run() {
  this->sense_and_send();
}


void setup() {

  // Initialize serial and wait for port to open:
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
 
  // DEBUG SIZE 
  // Serial.print("SIZE OF INT: ");
  // Serial.println(sizeof(int));

  // Serial.print("SIZE OF SHORT: ");
  // Serial.println(sizeof(short));

  // Serial.print("SIZE OF FLOAT: ");
  // Serial.println(sizeof(float));

  // Serial.print("SIZE OF MILLIS (TIME): ");
  // Serial.print(sizeof(unsigned long));
  // Serial.print(" MILLIS: ");
  // Serial.println(millis());

  // Serial.print("SIZE OF AUDIO BUF: ");
  // Serial.println(sizeof(stream_manager.sampleBuffer)); 
  // //

  stream_manager.blinkLED("green", 3);
 
  stream_manager.blinkLED("blue", 3);
  
  stream_manager.blinkLED("green", 3);

  stream_manager.initSensors();

  stream_manager.switchOnLED("green");

  delay(3000);

  stream_manager.switchOffLED("green");  
}

void loop() {

  stream_manager.run(); 
}

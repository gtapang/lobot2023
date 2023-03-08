
// Based on the RPLIDAR driver library provided by RoboPeak
// include library 
#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 14  // Define pin connected to MOTOCTL signal (used for controlling speed)
                          // For now, it's simple to use HIGH and LOW.

// -- For controlling speed --
// int pwmChannel = 0; // Selects channel 0
// int frequency = 1000; // PWM frequency of 1 KHz, user-defined
// int resolution = 8; // 8-bit resolution, 256 possible values
// int pwmPin = 14; // GPIO pin that will generate PWM signal

void setup() {

  // -- Controlling Motor Speed --
  // Configuration of channel 0 with the chosen freq and res
  // ledcSetup(pwmChannel, frequency, resolution);
  // Assign PWM channel to pin 14
  // ledcAttachPin(pwmPin, pwmChannel);

  Serial.begin(115200);
  // bind RPLIDAR driver to the esp32 hardware serial2
  Serial2.begin(115200, SERIAL_8N1, RX, TX);
  while(lidar.begin(Serial2));
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  delay(1000);
  // start rotation
  digitalWrite(RPLIDAR_MOTOR, HIGH);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle;   //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit;  //whether this point is belong to a new scan

    byte  quality  = lidar.getCurrentPoint().quality;   //quality of current measurement
    
    // perform data processing here...
    
    // Output the specified angle data
    if(angle > 0 and angle < 360 ){
      Serial.print(angle);
      Serial.print(" | ");
      Serial.println(distance);
    }
  } else {
    digitalWrite(RPLIDAR_MOTOR, LOW); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 360))) {
       // detected...
       lidar.startScan();

       // start motor rotating at maximum allowable speed
       digitalWrite(RPLIDAR_MOTOR, HIGH);
       delay(1000);
    }
  }
}

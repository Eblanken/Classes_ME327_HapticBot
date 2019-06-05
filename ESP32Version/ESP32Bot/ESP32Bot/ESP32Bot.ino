/*
 * File: esp32Bot
 * --------------
 * Erick Blankenberg, Beck Goodloe, Josiah Clark
 * ME210
 * 5/26/2019
 * Final
 *
 * Description:
 *   Teleoperated robot that streams lidar data and is controlled
 *   remotely.
 *  
 * Notes:
 *  - Make sure to install the husarnet ESP-32 arduino core to the hardware folder inside
 *    of your Arduino application under "/Contents/Java/hardware/esp32". You also need to run
 *    the get.py script to compile code.
 *  - Connecting to eduroam
 *    - You need to burn in your eduroam credentials with the companion script or provide them manually. Do NOT leave
 *     your student credentials in a git commit!!! Make sure to remove them from any comitted code (why EEPROM is preferred).
 *    - Heavily reccomended to install "EspExceptionDecoder" tool
 *    - Seems that there are some unaddressed errors with wpa_2 tracked at: https://github.com/espressif/arduino-esp32/issues/921
 *    - Appears that devices on husarnet are improperly blacklisted. I can ping the esp32 but I cannot
 *      do much more. I switched to using a UDP client and local access point.
 *    - Basically decided to just do wifi access point as the husarnet forum is dead and I am the first / last post on the 4-year old forum.
 *    - Seems that even with a direct access point that things are intermittent, I was able to communicate 
 *      to the device but attempting to send back a packet resulted in issues.
 *  - Connecting to LIDAR
 *    - Seems to just time out if motor is not running and the PWM pins I have
 *      been using all seem to stop? This is on the TeensyLC which has had some
 *      issues in the past and on the esp-32 whose PWM pin configuration is still a bit
 *      in the air
 *  - Connecting to motors         
 *    - See above PWM issue
 */

#include<WiFi.h>
#include<WiFiUdp.h>
#include<DNSServer.h>

#include <OSCMessage.h>
#include <OSCData.h>
#define BUILTIN_LED 2

/*
#include<HardwareSerial.h>
#include<RPLidar.h>

#include "Metro.h"
*/

/*------------------- Wifi Settings ---------------------*/

// UDP Client
WiFiUDP Udp;
DNSServer DnsServer;

// For personal hotspot
const byte DNS_PORT = 53;
const IPAddress local(11, 11, 11, 11);
const IPAddress dns(11, 11, 11, 11);
const IPAddress gateway(11, 11, 11, 11); // Unused
const IPAddress netmask(255, 255, 255, 0);

const unsigned int localPort = 8888;
OSCErrorCode error;
unsigned int ledState = LOW;

// Safety shutdown timer
/*
#define WATCHDOG_TIME 100
Metro WatchdogTimer = Metro(WATCHDOG_TIME);
*/

/*------------------- Motor Control ---------------------*/

typedef struct {
  int16_t upDownValue;
  int16_t leftRightValue;
} motorPacket;

typedef union {
  motorPacket internalPacket;
  uint8_t bufferRepresentation[sizeof(motorPacket)];
} motorPacketUnified;

// Utility
#define PIN_LED 2
//Metro Blinker = Metro(250);

// Motor control
#define PIN_MOTOR_1_A 10
#define PIN_MOTOR_1_B 11
#define PIN_MOTOR_2_A 12
#define PIN_MOTOR_2_B 13

/*------------------- Lidar Control ---------------------*/

typedef struct {
  float angle;
  float distance;
} lidarPacket;

typedef union {
  lidarPacket internalPacket;
  uint8_t bufferRepresentation[sizeof(lidarPacket)];
} lidarPacketUnified;

#define PIN_LIDAR_MOTOR  27
#define PIN_LIDAR_MAXVAL 255

//HardwareSerial LidarSerial(2);
//RPLidar Lidar;

/*-------------------- The Program ----------------------*/
void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  initializeWifi();
  //initializeLidar();
  //initializeMotors();
}

void loop() {
  //flashBlinker();
  
  // Comms
  //recieveMotorData();
  //sendLidarData();

  /*
  DnsServer.processNextRequest();  
  delay(0);
  */

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    Serial.println((String)"Request from " + Udp.remoteIP().toString() + ", port " + Udp.remotePort());
    
    Serial.print("Message: ");
    for(int currentVal = 0; currentVal < packetSize; currentVal++) {
      Serial.print((char) Udp.read());
    }
    Serial.println();
    */
    /*
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    uint8_t response[] = "ackowledged";
    Udp.write(response, sizeof(response));
    Udp.endPacket();
    */
  //}
}

//--------------------------- Motor Functions -----------------------------//

/*
void initializeMotors() {
  pinMode(PIN_MOTOR_1_A, OUTPUT);
  pinMode(PIN_MOTOR_1_B, OUTPUT);
  pinMode(PIN_MOTOR_2_A, OUTPUT);
  pinMode(PIN_MOTOR_2_B, OUTPUT);
  ledcSetup(1, 5000, 10);
  ledcSetup(2, 5000, 10);
  ledcSetup(3, 5000, 10);
  ledcSetup(4, 5000, 10);
  ledcAttachPin(PIN_MOTOR_1_A, 1);
  ledcAttachPin(PIN_MOTOR_1_B, 2);
  ledcAttachPin(PIN_MOTOR_2_A, 3);
  ledcAttachPin(PIN_MOTOR_2_B, 4);
  ledcWrite(PIN_MOTOR_1_A, 0);
  ledcWrite(PIN_MOTOR_1_B, 0);
  ledcWrite(PIN_MOTOR_2_A, 0);
  ledcWrite(PIN_MOTOR_2_B, 0);
}

void recieveMotorData() {
  static int currentUpDown    = 0;
  static int currentLeftRight = 0;
  // Gets most recent data 
  if(Udp.available()) {
    int packetSize = Udp.parsePacket();
    if(packetSize > 0) {
      Serial.print("Received Packet: ");
      Serial.println(packetSize);
      motorPacketUnified newestPacket;
      Udp.read(newestPacket.bufferRepresentation, sizeof(newestPacket.bufferRepresentation));
      Serial.println("Motor Packet: ");
      Serial.print(newestPacket.internalPacket.upDownValue);
      Serial.print(" ");
      Serial.println(newestPacket.internalPacket.leftRightValue);
      // Updates current position
      ledcWrite(PIN_MOTOR_1_A, (newestPacket.internalPacket.upDownValue >= 0) ? newestPacket.internalPacket.upDownValue : 0);
      ledcWrite(PIN_MOTOR_1_B, (newestPacket.internalPacket.upDownValue < 0) ? abs(newestPacket.internalPacket.upDownValue) : 0);
      ledcWrite(PIN_MOTOR_2_A, (newestPacket.internalPacket.leftRightValue >= 0) ? newestPacket.internalPacket.leftRightValue : 0);
      ledcWrite(PIN_MOTOR_2_B, (newestPacket.internalPacket.leftRightValue < 0) ? abs(newestPacket.internalPacket.leftRightValue) : 0);
    }
  }
  */
  /*
  if(WatchdogTimer.check()) {
    currentUpDown = 0;
    currentLeftRight = 0;
  }
  */
  /*
}

void flashBlinker() {
  if(Blinker.check()) {
    static bool lastLedState = false;
    digitalWrite(PIN_LED, lastLedState);
    lastLedState = !lastLedState;
    Blinker.reset();
  }
}
*/

//--------------------------- Lidar Functions -----------------------------//

/*
void initializeLidar() {
  // Motor
  pinMode(PIN_LIDAR_MOTOR, OUTPUT);
  ledcSetup(5, 5000, 10);
  ledcAttachPin(PIN_LIDAR_MOTOR, 5);
  ledcWrite(PIN_LIDAR_MOTOR, 0);
  // Lidar
  LidarSerial.begin(115200, SERIAL_8N1);
  Lidar.begin(LidarSerial);
}

void sendLidarData() {
  // Updates local data for latest point
  int result = Lidar.waitPoint();

  // Has a new point
  if( IS_OK(result)) {
    lidarPacketUnified latestPacket;
    latestPacket.internalPacket.distance = Lidar.getCurrentPoint().distance;
    latestPacket.internalPacket.angle    = Lidar.getCurrentPoint().angle;
    */
    /*
    Udp.beginPacket();
    Udp.printf("Seconds since boot: %u", millis()/1000);
    Udp.endPacket();
    */
    /*
    Udp.beginPacket();
    Udp.write(latestPacket.bufferRepresentation, sizeof(latestPacket.bufferRepresentation));
    Udp.endPacket();
    */
    /*
  // Has to reconnect to lidar
  } else {
    ledcWrite(PIN_LIDAR_MOTOR, 0); //stop the rplidar motor
    Serial.print("Lidar Not Connected: ");
    Serial.println(result, HEX);
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    u_result result = Lidar.getDeviceInfo(info, 100);
    Serial.print("Attempting to Connect, Result: ");
    Serial.println(result, HEX);
    if (IS_OK(result)) {
       Serial.println("Lidar Actually OK.");
       // detected...
       Lidar.startScan();
       // start motor rotating at max allowed speed
       ledcWrite(PIN_LIDAR_MOTOR, PIN_LIDAR_MAXVAL);
       delay(2000);
    }
  }
}
*/

//---------------------------- Wifi Functions -----------------------------//

// Starts up access point and udp server
void initializeWifi() {
  Serial.println("Setting Up Server");
  DnsServer.start(DNS_PORT, "*", local);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local, dns, netmask);
  WiFi.softAP("UDP-RECIEVER");
  WiFi.begin();
  
  Udp.begin(localPort); 
}



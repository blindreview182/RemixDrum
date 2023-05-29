
// Libraries
#include<Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// Constants 
#define touch 14 // touch sensor pin (D5 pin in ESP8266)
const int MPU=0x68; // MPU6050 accelerometer address
const IPAddress outIp(255,255,255,255); // COmputer IP that receive OSC messages 
const unsigned int outPort = 6666; // Pure Data Port 
const unsigned int outPort2 = 7778; // Processing Port
const unsigned int localPort = 2391; // Local Port

// Variables
char ssid[] = "remixdrum"; // EDIT: network name
char pass[] = "12345678"; // EDIT: network password
WiFiUDP Udp; // Instance that allow send and receive UDP package
int GyX, GyY, GyZ; // variables from axes X, Y, Z
int minVal=0; // minimum value of accelerometer MPU6050
int maxVal=65536; // maximum value of accelerometer MPU6050

void setup() {
  // baud rate
  Serial.begin(9600); 

  // Connect to WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  
  // pin mode for touch sensor
  pinMode(touch, INPUT);

  // MPU6050 setup 
  Wire.begin();
  Wire.beginTransmission(MPU); // Beggining the transmission
  Wire.write(0X6B);

  // Reading MPU6050
  Wire.write(0);
  Wire.endTransmission(true);
  
}

// touch sensor initial state 
int value = 0;

void loop() {
  
  // Starting Touch sensor 
  value = digitalRead(touch); // Reading touch sensor 
  Serial.println(value); // Print touch sensor value in serial monitor 
  delay(50); // Reading time (50 ms)

  // Starting MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0X3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true); // MPU6050 data (14 bytes)
  // Lendo dados do giroscópio
  GyX=Wire.read()<<8|Wire.read();  //0x3B (GYRO_XOUT_H) & 0x3C (GYRO_XOUT_L)     
  GyY=Wire.read()<<8|Wire.read();  //0x3D (GYRO_YOUT_H) & 0x3E (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  //0x3F (GYRO_ZOUT_H) & 0x40 (GYRO_ZOUT_L)

  // Accelerometer range
  int xAng = map(GyX,minVal,maxVal,1,5);
  int yAng = map(GyY,minVal,maxVal,0,180);
  int zAng = map(GyZ,minVal,maxVal,80,130);

   // Print X axe in serial monitor
  Serial.print(" | GyX = "); Serial.print(xAng);
  // Print Y axe in serial monitor
  Serial.print(" | GyY = "); Serial.print(yAng);
  // Print Z axe in serial monitor
  Serial.print(" | GyZ = "); Serial.println(zAng);
  // Time between print 
  delay(300);

 // OSC message from touch sensor
  OSCMessage botao("/value");
  botao.add((int32_t)int(value));
  Udp.beginPacket(outIp, outPort);
  botao.send(Udp);
  Udp.endPacket();
  botao.empty();
  delay(500);

  // Osc Message MPU6050 to Pure Data
  OSCMessage axleX("/gyx");
  axleX.add((int32_t)int(xAng));
  Udp.beginPacket(outIp, outPort);
  axleX.send(Udp);
  Udp.endPacket();
  axleX.empty();
  delay(10);

  OSCMessage processingaxleX("/gyx");
  processingaxleX.add((int32_t)int(xAng));
  Udp.beginPacket(outIp, outPort2);
  processingaxleX.send(Udp);
  Udp.endPacket();
  processingaxleX.empty();
  delay(10);

 
  OSCMessage axleY("/gyy");
  axleY.add((int32_t)int(yAng));
  Udp.beginPacket(outIp, outPort);
  axleY.send(Udp);
  Udp.endPacket();
  axleY.empty();
  delay(10);

  // 
  // Osc Message MPU6050 to Processing
  OSCMessage processingaxleY("/gyy");
  processingaxleY.add((int32_t)int(yAng));
  Udp.beginPacket(outIp, outPort2);
  processingaxleY.send(Udp);
  Udp.endPacket();
  processingaxleY.empty();
  delay(10);

 
  OSCMessage axleZ("/gyz");
  axleZ.add((int32_t)int(zAng));
  Udp.beginPacket(outIp, outPort);
  axleZ.send(Udp);
  Udp.endPacket();
  axleZ.empty();
  delay(100);

  OSCMessage processingaxleZ("/gyz");
  processingaxleZ.add((int32_t)int(zAng));
  Udp.beginPacket(outIp, outPort);
  processingaxleZ.send(Udp);
  Udp.endPacket();
  processingaxleZ.empty();
  delay(100);

}

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include<Wire.h>

//#include <UbidotsMicroESP8266.h>
//#define TOKEN "XXXXXXXXXXXXXXXXXXXXXX" // Put here your Ubidots TOKEN
char * ssid = "cashew"; // Put your WiFi SSID here
char * pass = "oW329bTdVmYJ"; // Put your Wifi Password here
//Ubidots client(TOKEN);

const int MPU_addr = 0x68; // I2C address of the MPU-6050

WiFiUDP udp;
boolean wifiConnected = false;
//int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
IPAddress ip (10,1,1,110); // the remote IP address
const unsigned int port = 12345;      // port to send UDP packets
int status = WL_IDLE_STATUS;
uint32_t counter;

int minVal=265; int maxVal=402;

struct MPUtelemetry {
  char magic[5];
  unsigned long counter;
  unsigned long ms;
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
};

MPUtelemetry tm = {"tele",0,0,0,0,0,0,0,0};

void setup() {
  //client.wifiConnection(WIFISSID, PASSWORD);
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200);

  Serial.print("Packet size: ");
  Serial.println(sizeof(MPUtelemetry));

// Connect to WiFi network
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

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  udp.begin(port);
  counter = 0;
}

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  tm.AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  tm.AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  tm.AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tm.Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  tm.GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  tm.GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  tm.GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("# = "); Serial.print(counter);
  Serial.print("| ms = "); Serial.print(millis());
  //Serial.print(" | Packet size: ");
  //Serial.print(sizeof(MPUtelemetry));
  Serial.print(" | AcX = "); Serial.print(tm.AcX);
  Serial.print(" | AcY = "); Serial.print(tm.AcY);
  Serial.print(" | AcZ = "); Serial.print(tm.AcZ);
  Serial.print(" | Tmp = "); Serial.print(tm.Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(tm.GyX);
  Serial.print(" | GyY = "); Serial.print(tm.GyY);
  Serial.print(" | GyZ = "); Serial.println(tm.GyZ);
  tm.counter = counter;
  counter++;

  //Wire.beginTransmission(MPU_addr); Wire.write(0x3B); Wire.endTransmission(false); Wire.requestFrom(MPU_addr,14,true); AcX=Wire.read()<<8|Wire.read(); AcY=Wire.read()<<8|Wire.read(); AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(tm.AcX,minVal,maxVal,-90,90);
  int yAng = map(tm.AcY,minVal,maxVal,-90,90);
  int zAng = map(tm.AcZ,minVal,maxVal,-90,90);

  double x; double y; double z;
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  Serial.print("Angles X= "); Serial.print(x);
  Serial.print(" Y= "); Serial.print(y);
  Serial.print(" Z= "); Serial.println(z); 

  //Serial.println(AcX);

  //client.add("Tilt", AcX);
  //client.sendAll(true);
  tm.ms = millis();
  //char *message = "Hello from wemos D1 mini!";
  udp.beginPacket(ip,port);
  udp.write((char *)&tm, sizeof(tm));
  udp.endPacket();

  delay(40);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

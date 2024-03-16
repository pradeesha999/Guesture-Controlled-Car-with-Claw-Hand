#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ezButton.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"   

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t devStatus;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

ezButton toggleSwitch(4);

const uint64_t pipeOut = 0xF9E8F0F0E1LL;
RF24 radio(8, 9); // select CE,CSN pin


struct PacketData
{
  float x1;
  float y1;
  float z1;
  int m1;
  int s1;
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
} data;

void setupRadioTransmitter()
{
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.stopListening(); //start the radio comunication for Transmitter  

  data.xAxisValue = 127; // Center
  data.yAxisValue = 127; // Center 
  data.zAxisValue = 127;
}

void setupMPU()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

const int MPU_addr = 0x68;

int16_t axis_X, axis_Y, axis_Z;

int minVal = 265;
int maxVal = 402;

float x;
float y;
float z;
int m;
int s;

int reed = 7;

void setup() {

  Serial.begin(2000000);
  pinMode(reed,INPUT);
  toggleSwitch.setDebounceTime(10);
  setupRadioTransmitter();
  setupMPU();

  Wire.begin();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

}

void loop() {
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  
  toggleSwitch.loop();
  int state = toggleSwitch.getState();
  if (state == LOW)
  {

    s = 1;
    
    if (!dmpReady)
    return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
      int xAxisValue = constrain(ypr[2] * 180 / M_PI, -90, 90);
      int yAxisValue = constrain(ypr[1] * 180 / M_PI, -90, 90);
      int zAxisValue = constrain(ypr[0] * 180 / M_PI, -90, 90);
  
      data.xAxisValue = map(xAxisValue, -90, 90, 0, 254);
      data.yAxisValue = map(yAxisValue, -90, 90, 254, 0);
      data.zAxisValue = map(zAxisValue, -90, 90, 0, 254);
      data.s1 = s;
  
      radio.write(&data, sizeof(PacketData));
  
      Serial.print("xAxis: ");
      Serial.print(xAxisValue);
      Serial.print("\t");
      Serial.print("yAxis: ");
      Serial.print(yAxisValue);
      Serial.print("\t");
      Serial.print("zAxis: ");
      Serial.println(zAxisValue);

      Serial.println("Switch: Car ");
      Serial.print("Switch Status : ");
      Serial.println(data.s1);

    }
  }
    
  else
  {
    s = 0;
    
    axis_X = Wire.read() << 8 | Wire.read();
    axis_Y = Wire.read() << 8 | Wire.read();
    axis_Z = Wire.read() << 8 | Wire.read();
  
    int xAng = map(axis_X*0.5, minVal, maxVal, 90, -90);
    int yAng = map(axis_Y*0.5, minVal, maxVal, -90, 90);
    int zAng = map(axis_Z, minVal, maxVal, -90, 90);
  
    x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  
    
    int magstat = digitalRead(reed);
    if(magstat == HIGH)
    {
      m = 100;
    }
    else
    {
      m = 150;
    }
    
    data.x1 = x;
    data.y1 = y;
    data.z1 = z;
    data.m1 = m;
    data.s1 = s;
  
    Serial.print("Calculated: ");
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("\t\t");
    Serial.print("y: ");
    Serial.print(y);
    Serial.print("\t\t");
    Serial.print("z: ");
    Serial.println(z);
  
    Serial.print("Transmitted: ");
    Serial.print("x: ");
    Serial.print(data.x1);
    Serial.print("\t\t");
    Serial.print("y: ");
    Serial.print(data.y1);
    Serial.print("\t\t");
    Serial.print("z: ");
    Serial.println(data.z1);
  
    Serial.print("Grip: ");
    Serial.println(data.m1);

    Serial.println("Switch: Claw ");
    
    Serial.print("Switch Status :");
    Serial.println(data.s1);
  
    radio.write(&data, sizeof(PacketData));
    delay(10);
  }
}

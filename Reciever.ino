  #include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<Servo.h>


#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds.
#define MAX_MOTOR_SPEED 130

const uint64_t pipeIn = 0xF9E8F0F0E1LL;
RF24 radio(8, 9);
unsigned long lastRecvTime = 0;

struct PacketData
{
  float x1;
  float y1;
  float z1;
  int m1;
  int s1;
  byte xAxisValue;
  byte yAxisValue;
} receiverData;

//Right motor
int enableRightMotor = 10;
int rightMotorPin1 = A0;
int rightMotorPin2 = A1;

//Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = A2;
int leftMotorPin2 = A3;


Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;

float x;
float y;
float z;
int m;
int s;

int minVal = 265;
int maxVal = 402;


void setup() {

  Serial.begin(115200);

  servo_1.attach(3);   // Forward/Reverse_Motor
  servo_2.attach(5);   // Up/Down_Motor
  servo_3.attach(4);   // Gripper_Motor
  servo_4.attach(2);   // Left/Right_Motor


  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  rotateMotor(0, 0);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();

}

void loop() {

  
  if (radio.available())
  {
    radio.read(&receiverData, sizeof(PacketData));
    if (receiverData.s1 == 0)
    {
      Serial.print("Received: ");
      Serial.print("x: ");
      Serial.print(receiverData.x1);
      Serial.print("\t");
      Serial.print("y: ");
      Serial.print(receiverData.y1);
      Serial.print("\t");
      Serial.print("z: ");
      Serial.println(receiverData.z1);

      Serial.print("grip :");
      Serial.println(receiverData.m1);

      Serial.println("Switch : Claw");

      Serial.print("Switch Status : ");
      Serial.println(receiverData.s1);

      x = receiverData.x1;
      y = receiverData.y1;
      z = receiverData.z1;
      m = receiverData.m1;

      if (x >= 0 && x <= 30)
      {
 
        int mov1 = map(x, 0, 30, 90, 180); 
        //int mov2 = map(x, 0, 60, 90, 180); 
        //servo_2.write(mov2);
        servo_1.write(mov1); 
         
      }

      if (x >= 30 && x <= 60)
      {
 
        //int mov1 = map(x, 0, 60, 90, 180); 
        int mov2 = map(x, 30, 60, 90, 180); 
        servo_2.write(mov2);
        //servo_1.write(mov1); 
         
      }
      else if (x >= 300 && x <= 330)
      {
        int mov1 = map(x, 300, 330, 180, 90); 
        //int mov2 = map(x, 300, 360, 0, 90); 
        servo_1.write(mov1); 
        //servo_2.write(mov2); 
      }

      else if (x >= 330 && x <= 360)
      {
        //int mov1 = map(x, 300, 360, 180, 90); 
        int mov2 = map(x, 330, 360, 0, 90); 
        //servo_1.write(mov1); 
        servo_2.write(mov2); 
      }


      if (y >= 0 && y <= 60)
      {
        int mov3 = map(y, 0, 60, 90, 180);
        //      Serial.print("Movement in Left = ");
        //      Serial.print(mov3);
        //      Serial.println((char)176);
        servo_4.write(mov3);
      }


      else if (y >= 300 && y <= 360)
      {
        int mov3 = map(y, 300, 360, 0, 90);
        //      Serial.print("Movement in Right = ");
        //      Serial.print(mov3);
        //      Serial.println((char)176);
        servo_4.write(mov3);
      }

      servo_3.write(m);

    }
    else
    {
      int rightMotorSpeed = 0;
      int leftMotorSpeed = 0;

      if (receiverData.xAxisValue <= 85)       //Move car Forward
      {
        rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }
      else if (receiverData.xAxisValue >= 165)   //Move car Backward
      {
        rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
      }
      else if (receiverData.yAxisValue <= 85)  //Move car Right
      {
        rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }
      else if (receiverData.yAxisValue >= 165)   //Move car Left
      {
        rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
      }
      else                                      //Stop the car
      {
        rotateMotor(0, 0);
      } 
      lastRecvTime = millis();

      Serial.println(receiverData.xAxisValue);
      Serial.println(receiverData.yAxisValue);
    }
  }
  else
  {
    
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

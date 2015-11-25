//
// ROSRomeoNode - Implements a ROS node for the RomeoBLE for controlling
//     a ROSRev2-class robot.
//

// Needed on Leonardo to force use of USB serial.
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

const int ONBOARD_SWITCH_PIN = A7;
const int ONBOARD_LED_PIN = 13;

// The pins for motor control on the Romeo BLE.
const int M1_DIRECTION = 4;
const int M1_SPEED = 5;
const int M2_SPEED = 6;
const int M2_DIRECTION = 7;

// Pins for the Pololu motor encoder outputs.
const int M1_A = 2;
const int M1_B = 8;
const int M2_A = 3;
const int M2_B = 9;

ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelRateMsg;
ros::Publisher lwheelRatePub("lwheel_rate", &lwheelRateMsg);

std_msgs::Float32 rwheelRateMsg;
ros::Publisher rwheelRatePub("rwheel_rate", &rwheelRateMsg);

void lmotorCmdCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lmotorCmdSub("lmotor_cmd", &lmotorCmdCallback);

void rmotorCmdCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rmotorCmdSub("rmotor_cmd", &rmotorCmdCallback);

volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

float lwheelRate = 0.0;
float rwheelRate = 0.0;

int lmotorCmd = 0;
int rmotorCmd = 0;

int lmotorMin;
int rmotorMin;

int motorCmdTimeout;

unsigned long lastLoopTime;
unsigned long lastMotorCmdTime;

void setup()
{
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(M1_DIRECTION, OUTPUT);
  pinMode(M2_DIRECTION, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(M1_A), leftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_A), rightAChange, CHANGE);
  
  nh.initNode();

  float motorMin;
  nh.getParam("~lmotorMin", &motorMin);
  lmotorMin = (int) motorMin;
  nh.getParam("~rmotorMin", &motorMin);
  rmotorMin = (int) motorMin;

  nh.getParam("~motor_cmd_timeout", &motorCmdTimeout);

  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;
}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  long curLoopTime = micros();

  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  lwheelMsg.data = (int) curLwheel;
  rwheelMsg.data = (int) curRwheel;
  lwheelPub.publish(&lwheelMsg);
  rwheelPub.publish(&rwheelMsg);

  unsigned long deltaMicros = curLoopTime - lastLoopTime;

  lwheelRate = 0.5*lwheelRate + 0.5*((curLwheel - lastLwheel) / (float) deltaMicros * 1E6);
  rwheelRate = 0.5*rwheelRate + 0.5*((curRwheel - lastRwheel) / (float) deltaMicros * 1E6);
  lwheelRateMsg.data = lwheelRate;
  rwheelRateMsg.data = rwheelRate;
  lwheelRatePub.publish(&lwheelRateMsg);
  rwheelRatePub.publish(&rwheelRateMsg);
  
  lastLwheel = curLwheel;
  lastRwheel = curRwheel;
  
  // Turn off motors if too much time has elapsed since last motor command.
  if (millis() - lastMotorCmdTime > motorCmdTimeout) {
    lmotorCmd = 0;
    rmotorCmd = 0;
    setSpeed(0, 0);
  }

  nh.spinOnce();
  delay(10);
}

void lmotorCmdCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  lmotorCmd = cmdMsg.data;

  setSpeed(lmotorCmd, rmotorCmd);
}

void rmotorCmdCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  rmotorCmd = cmdMsg.data;

  setSpeed(lmotorCmd, rmotorCmd);
}

void leftAChange() {
  if (digitalRead(M1_A) == digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void rightAChange() {
  if (digitalRead(M2_A) != digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

// Sets the left and right motor speeds.
void setSpeed(int leftSpeed, int rightSpeed) {
  if (abs(leftSpeed) < lmotorMin) {
    leftSpeed = 0;
  }
  if (abs(rightSpeed) < rmotorMin) {
    rightSpeed = 0;
  }

  digitalWrite(M1_DIRECTION, (leftSpeed >= 0 ? HIGH : LOW));
  analogWrite(M1_SPEED, abs(leftSpeed));
  digitalWrite(M2_DIRECTION, (rightSpeed >= 0 ? HIGH : LOW));
  analogWrite(M2_SPEED, abs(rightSpeed));
}


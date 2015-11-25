// Needed on Leonardo to force use of USB serial.
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 leftTicksMsg;
ros::Publisher leftEncoder("left_wheel", &leftTicksMsg);

std_msgs::Int32 rightTicksMsg;
ros::Publisher rightEncoder("right_wheel", &rightTicksMsg);

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void setup()
{
  attachInterrupt(digitalPinToInterrupt(2), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightTick, CHANGE);
  
  nh.initNode();
  nh.advertise(leftEncoder);
  nh.advertise(rightEncoder);
}

void loop()
{
  noInterrupts();
  leftTicksMsg.data = leftTicks;
  rightTicksMsg.data = rightTicks;
  interrupts();
  
  leftEncoder.publish(&leftTicksMsg);
  rightEncoder.publish(&rightTicksMsg);
  
  nh.spinOnce();
  delay(1000);
}

void leftTick() {
  if (digitalRead(2) == digitalRead(8)) {
    ++leftTicks;
  } else {
    --leftTicks;
  }
}

void rightTick() {
  if (digitalRead(3) != digitalRead(9)) {
    ++rightTicks;
  } else {
    --rightTicks;
  }
}


//
// ROSRomeoNode - Implements a ROS node for the RomeoBLE for controlling
//     a ROSRev2-class robot.
//

// Needed on Leonardo to force use of USB serial.
//#define USE_USBCON

#include <AStar32U4.h>
#include <EnableInterrupt.h>
#include <SimplePID.h>
#include <TaskManager.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

class StringBuf : public Print {
  private:
    int len;
    char msg[80];
    
  public:
    StringBuf() {
      len = 0;
    }
    
    size_t write(uint8_t c) {
      if (len < sizeof(msg)-1) {
        msg[len++] = c;
        msg[len] = '\0';
      }

      return 1;
    }

    const char *get() {
      msg[sizeof(msg)-1] = '\0';
      len = 0;
      return msg;
    }

    StringBuf& print_P(const char *s) {
      int c;
      while ((c = pgm_read_byte(s++)) != '\0') {
        write(c);
      }
    }
    
};

StringBuf buf;

const int ONBOARD_LED_PIN = 13;

// Pins for the A-Star motor encoder outputs.
const int M1_A = 7;
const int M1_B = 11;
const int M2_A = 15;
const int M2_B = 16;

ros::NodeHandle_<ArduinoHardware, 2, 13, 125, 125> nh;

std_msgs::Int32 int32Msg;
std_msgs::Float32 float32Msg;

ros::Publisher lwheelPub("~lwheel", &float32Msg);

ros::Publisher rwheelPub("~rwheel", &float32Msg);

ros::Publisher lwheelVelocityPub("~lwheel_vel", &float32Msg);

ros::Publisher rwheelVelocityPub("~rwheel_vel", &float32Msg);

ros::Publisher a0Pub("~a0", &int32Msg);
ros::Publisher a2Pub("~a2", &int32Msg);
ros::Publisher a3Pub("~a3", &int32Msg);
ros::Publisher a4Pub("~a4", &int32Msg);
ros::Publisher a5Pub("~a5", &int32Msg);

ros::Publisher batteryVoltagePub("~battery_voltage", &float32Msg);

ros::Publisher buttonAPub("~button_a", &int32Msg);
ros::Publisher buttonBPub("~button_b", &int32Msg);
ros::Publisher buttonCPub("~button_c", &int32Msg);

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("~lwheel_vtarget", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("~rwheel_vtarget", &rwheelTargetCallback);

AStar32U4Motors motors;

// These objects provide access to the A-Star's on-board
// buttons.  We will only use buttonA.
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning

const float Ku = .15;
const float Tu = .1142857143;

const float Kp = 0.6 * Ku;
const float Ki = 2 * Kp / Tu;
const float Kd = Kp * Tu / 8;

SimplePID leftController = SimplePID(Kp, Ki, Kd);
SimplePID rightController = SimplePID(Kp, Ki, Kd);

volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

// Target motors speeds in ticks per second.
int lwheelTargetRate = 0;
int rwheelTargetRate = 0;

// The interval between motor control steps.
int controlRate;
int controlDelayMillis;

// The number of encoder ticks per meter.
int ticksPerMeter;

// The number of milliseconds without a velocity target when the robot
// will automatically turn off the motors.
float vtargetTimeout;
int vtargetTimeoutMillis;

unsigned long lastLoopTime;
unsigned long lastMotorCmdTime;

int leftMotorCmd = 0;
int rightMotorCmd = 0;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 60;

// Maximum motor control value.
const int MAX_MOTOR_CMD = 400;

TaskManager taskMgr;

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  enableInterrupt(M1_A, leftAChange, CHANGE);
  enableInterrupt(M1_B, leftBChange, CHANGE);
  enableInterrupt(M2_A, rightAChange, CHANGE);
  enableInterrupt(M2_B, rightBChange, CHANGE);

  nh.initNode();

  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);

  nh.advertise(a0Pub);
  nh.advertise(a2Pub);
  nh.advertise(a3Pub);
  nh.advertise(a4Pub);
  nh.advertise(a5Pub);

  nh.advertise(batteryVoltagePub);

  nh.advertise(buttonAPub);
  nh.advertise(buttonBPub);
  nh.advertise(buttonCPub);

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);

  // Wait until the node has initialized before getting parameters.
  while (!nh.connected()) {
    nh.spinOnce();
  }

  buf.print_P(PSTR("Test message: "));
  buf.print(123);
  nh.loginfo(buf.get());

  buf.print_P(PSTR("~control_rate"));
  if (nh.getParam(buf.get(), &controlRate)) {
    buf.print_P(PSTR("~control_rate = "));
    buf.print(controlRate);
  } else {
    controlRate = 60;
    buf.print_P(PSTR("~control_rate defaulting to "));
    buf.print(controlRate);
  }
  nh.loginfo(buf.get());
  nh.spinOnce();
  controlDelayMillis = 1000.0 / controlRate;

  buf.print_P(PSTR("~ticks_meter"));
  if (nh.getParam(buf.get(), &ticksPerMeter)) {
    buf.print_P(PSTR("~ticks_meter = "));
    buf.print(ticksPerMeter);
  } else {
    ticksPerMeter = 11931;
    buf.print_P(PSTR("~ticks_meter defaulting to "));
    buf.print(ticksPerMeter);
  }
  nh.loginfo(buf.get());
  nh.spinOnce();

  buf.print_P(PSTR("~vtarget_timeout"));
  if (nh.getParam(buf.get(), &vtargetTimeout)) {
    buf.print_P(PSTR("~vtarget_timeout = "));
    buf.print(vtargetTimeout);
  } else {
    vtargetTimeout = 0.250;
    buf.print_P(PSTR("~vtarget_timeout defaulting to "));
    buf.print(vtargetTimeout);
  }
  nh.loginfo(buf.get());
  nh.spinOnce();
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;

  taskMgr.addPeriodicTask(controlMotors, controlDelayMillis);
  taskMgr.addPeriodicTask(publishAnalogValues, 17); // 17ms ~ 60Hz
  taskMgr.addPeriodicTask(publishSwitches, 100); // 100ms ~ 10Hz
  taskMgr.addPeriodicTask(publishVoltage, 1000); // 1000ms ~ 1Hz
}

// Every loop, publish the encoder and wheel rates.
void loop() {
  taskMgr.doTasks();
  nh.spinOnce();
  delay(1);
}

void controlMotors() {
  long curLoopTime = micros();

  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  float32Msg.data = (float) curLwheel / ticksPerMeter;
  lwheelPub.publish(&float32Msg);
  float32Msg.data = (float) curRwheel / ticksPerMeter;
  rwheelPub.publish(&float32Msg);

  float dt = (curLoopTime - lastLoopTime) / 1E6;

  float lwheelRate = ((curLwheel - lastLwheel) / dt);
  float rwheelRate = ((curRwheel - lastRwheel) / dt);

  float32Msg.data = lwheelRate / ticksPerMeter;
  lwheelVelocityPub.publish(&float32Msg);
  float32Msg.data = rwheelRate / ticksPerMeter;
  rwheelVelocityPub.publish(&float32Msg);

  int leftControl = leftController.getControlValue(lwheelRate, dt);
  leftMotorCmd += min(MAX_MOTOR_CMD, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  if (leftMotorCmd > 0) {
    leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
  }

  int rightControl = rightController.getControlValue(rwheelRate, dt);
  rightMotorCmd += min(MAX_MOTOR_CMD, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  if (rightMotorCmd > 0) {
    rightMotorCmd = max(rightMotorCmd, MIN_MOTOR_CMD);
  }

  // Coast to a stop if target is zero.
  if (lwheelTargetRate == 0) {
    leftMotorCmd = 0;
  }
  if (rwheelTargetRate == 0) {
    rightMotorCmd = 0;
  }

  setSpeed(leftMotorCmd, rightMotorCmd);

  // Turn off motors if too much time has elapsed since last motor command.
  if (millis() - lastMotorCmdTime > vtargetTimeoutMillis) {
    lwheelTargetRate = 0;
    rwheelTargetRate = 0;
    setSpeed(0, 0);
  }

  lastLwheel = curLwheel;
  lastRwheel = curRwheel;

  lastLoopTime = curLoopTime;
}

void publishAnalogValues() {
  int a0 = analogRead(A0);
  int a2 = analogRead(A2);
  int a3 = analogRead(A3);
  int a4 = analogRead(A4);
  int a5 = analogRead(A5);

  int32Msg.data = a0;
  a0Pub.publish(&int32Msg);

  int32Msg.data = a2;
  a2Pub.publish(&int32Msg);

  int32Msg.data = a3;
  a3Pub.publish(&int32Msg);

  int32Msg.data = a4;
  a4Pub.publish(&int32Msg);

  int32Msg.data = a5;
  a5Pub.publish(&int32Msg);
}

void publishSwitches() {
  int32Msg.data = buttonA.getSingleDebouncedPress();
  buttonAPub.publish(&int32Msg);
}

void publishVoltage() {
  float32Msg.data = readBatteryMillivoltsLV() / 1000.0;
  batteryVoltagePub.publish(&float32Msg);
}

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  lwheelTargetRate = cmdMsg.data * ticksPerMeter;
  leftController.setSetPoint(lwheelTargetRate);
}

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  rwheelTargetRate = cmdMsg.data * ticksPerMeter;
  rightController.setSetPoint(rwheelTargetRate);
}

void leftAChange() {
  if (digitalRead(M1_A) == digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void leftBChange() {
  if (digitalRead(M1_A) != digitalRead(M1_B)) {
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

void rightBChange() {
  if (digitalRead(M2_A) == digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

// Sets the left and right motor speeds.
void setSpeed(int leftSpeed, int rightSpeed) {
  motors.setSpeeds(leftSpeed, rightSpeed);
}


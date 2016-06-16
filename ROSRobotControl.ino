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
#include <std_msgs/Int16.h>
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

ros::NodeHandle_<ArduinoHardware, 5, 14, 125, 125> nh;

std_msgs::Int16 int16Msg;
std_msgs::Float32 float32Msg;

ros::Publisher lwheelPub("~l_ticks", &int16Msg);
ros::Publisher lwheelVelocityPub("~l_vel", &float32Msg);

ros::Publisher rwheelPub("~r_ticks", &int16Msg);
ros::Publisher rwheelVelocityPub("~r_vel", &float32Msg);

ros::Publisher a0Pub("~a0", &int16Msg);
ros::Publisher a2Pub("~a2", &int16Msg);
ros::Publisher a3Pub("~a3", &int16Msg);
ros::Publisher a4Pub("~a4", &int16Msg);
ros::Publisher a5Pub("~a5", &int16Msg);
ros::Publisher a6Pub("~a6", &int16Msg);

ros::Publisher batteryVoltagePub("~bat_v", &float32Msg);

ros::Publisher buttonAPub("~btn_a", &int16Msg);
ros::Publisher buttonBPub("~btn_b", &int16Msg);
ros::Publisher buttonCPub("~btn_c", &int16Msg);

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("~l_vtarg", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("~r_vtarg", &rwheelTargetCallback);

void ledGreenCallback(const std_msgs::Int16& cmdMsg);
ros::Subscriber<std_msgs::Int16> ledGreenSub("~led_g", &ledGreenCallback);

void ledRedCallback(const std_msgs::Int16& cmdMsg);
ros::Subscriber<std_msgs::Int16> ledRedSub("~led_r", &ledRedCallback);

void ledYellowCallback(const std_msgs::Int16& cmdMsg);
ros::Subscriber<std_msgs::Int16> ledYellowSub("~led_y", &ledYellowCallback);

AStar32U4Motors motors;

// These objects provide access to the A-Star's on-board
// buttons.  We will only use buttonA.
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

int lastButtonA = 0;
int lastButtonB = 0;
int lastButtonC = 0;

const float DEFAULT_Kp = 0.1;
const float DEFAULT_Kd = 0.0;
const float DEFAULT_Ki = 0.0;

float Kp = DEFAULT_Kp;
float Ki = DEFAULT_Ki;
float Kd = DEFAULT_Kd;

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
int controlDelayMillis;

// The number of milliseconds without a velocity target when the robot
// will automatically turn off the motors.
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
  nh.advertise(a6Pub);

  nh.advertise(batteryVoltagePub);

  nh.advertise(buttonAPub);
  nh.advertise(buttonBPub);
  nh.advertise(buttonCPub);

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);
  nh.subscribe(ledGreenSub);
  nh.subscribe(ledRedSub);
  nh.subscribe(ledYellowSub);

  // Wait until the node has initialized before getting parameters.
  while (!nh.connected()) {
    nh.spinOnce();
  }

  int controlRate;
  buf.print_P(PSTR("~control_rate"));
  if (nh.getParam(buf.get(), &controlRate)) {
    buf.print_P(PSTR("~control_rate = "));
  } else {
    buf.print_P(PSTR("~control_rate defaulting to "));
    controlRate = 50;
  }
  buf.print(controlRate);
  buf.print_P(PSTR("Hz"));
  nh.loginfo(buf.get());
  nh.spinOnce();
  controlDelayMillis = 1000.0 / controlRate;

  float vtargetTimeout;
  buf.print_P(PSTR("~vtarget_timeout"));
  if (nh.getParam(buf.get(), &vtargetTimeout)) {
    buf.print_P(PSTR("~vtarget_timeout = "));
  } else {
    buf.print_P(PSTR("~vtarget_timeout defaulting to "));
    vtargetTimeout = 0.5;
  }
  buf.print(vtargetTimeout, 3);
  nh.loginfo(buf.get());
  nh.spinOnce();
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  float param;
  
  buf.print_P(PSTR("~k_p"));
  if (nh.getParam(buf.get(), &param)) {
    buf.print_P(PSTR("~k_p = "));
    Kp = param;
  } else {
    buf.print_P(PSTR("~k_p defaulting to "));
    Kp = DEFAULT_Kp;
  }
  buf.print(Kp, 3);
  nh.loginfo(buf.get());
  nh.spinOnce();

  buf.print_P(PSTR("~k_i"));
  if (nh.getParam(buf.get(), &param)) {
    buf.print_P(PSTR("~k_i = "));
    Ki = param;
  } else {
    buf.print_P(PSTR("~k_i defaulting to "));
    Ki = DEFAULT_Ki;
  }
  buf.print(Ki, 3);
  nh.loginfo(buf.get());
  nh.spinOnce();

  buf.print_P(PSTR("~k_d"));
  if (nh.getParam(buf.get(), &param)) {
    buf.print_P(PSTR("~k_d = "));
    Kd = param;
  } else {
    buf.print_P(PSTR("~k_d defaulting to "));
    Kd = DEFAULT_Kd;
  }
  buf.print(Kd, 3);
  nh.loginfo(buf.get());
  nh.spinOnce();
  
  leftController.setConstants(Kp, Ki, Kd);
  rightController.setConstants(Kp, Ki, Kd);

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;

  taskMgr.addPeriodicTask(controlMotors, controlDelayMillis);
  taskMgr.addPeriodicTask(publishAnalogValues, 17); // 17ms ~ 60Hz
  taskMgr.addPeriodicTask(publishSwitches, 50); // 50ms ~ 20Hz
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

  int16Msg.data = curLwheel;
  lwheelPub.publish(&int16Msg);
  int16Msg.data = curRwheel;
  rwheelPub.publish(&int16Msg);

  float dt = (curLoopTime - lastLoopTime) / 1E6;

  float lwheelRate = ((curLwheel - lastLwheel) / dt);
  float rwheelRate = ((curRwheel - lastRwheel) / dt);

  float32Msg.data = lwheelRate;
  lwheelVelocityPub.publish(&float32Msg);
  float32Msg.data = rwheelRate;
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

  nh.spinOnce();
}

void publishAnalogValue(int pin, ros::Publisher *pub) {
  int16Msg.data = analogRead(pin);
  pub->publish(&int16Msg);
}

void publishAnalogValues() {
  publishAnalogValue(A0, &a0Pub);
  publishAnalogValue(A2, &a2Pub);
  publishAnalogValue(A3, &a3Pub);
  publishAnalogValue(A4, &a4Pub);
  publishAnalogValue(A5, &a5Pub);
  publishAnalogValue(A6, &a6Pub);

  nh.spinOnce();
}

void checkSwitch(int &state, PushbuttonBase *button, ros::Publisher *pub) {
  if (!state && button->getSingleDebouncedPress()) {
    int16Msg.data = 1;
    pub->publish(&int16Msg);
    state = 1;
  } else if (state && button->getSingleDebouncedRelease()) {
    int16Msg.data = 0;
    pub->publish(&int16Msg);
    state = 0;
  }
}

void publishSwitches() {
  checkSwitch(lastButtonA, &buttonA, &buttonAPub);
  checkSwitch(lastButtonB, &buttonB, &buttonBPub);
  checkSwitch(lastButtonC, &buttonC, &buttonCPub);

  nh.spinOnce();
}

void publishVoltage() {
  float32Msg.data = readBatteryMillivoltsLV() / 1000.0;
  batteryVoltagePub.publish(&float32Msg);

  nh.spinOnce();
}

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  lwheelTargetRate = cmdMsg.data;
  leftController.setSetPoint(lwheelTargetRate);
}

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  rwheelTargetRate = cmdMsg.data;
  rightController.setSetPoint(rwheelTargetRate);
}

void ledGreenCallback(const std_msgs::Int16& cmdMsg) {
  ledGreen(cmdMsg.data != 0);
}

void ledRedCallback(const std_msgs::Int16& cmdMsg) {
  ledRed(cmdMsg.data != 0);
}

void ledYellowCallback(const std_msgs::Int16& cmdMsg) {
  ledYellow(cmdMsg.data != 0);
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


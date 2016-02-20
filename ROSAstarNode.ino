//
// ROSAstarNode - Implements a ROS node for the Astar32U4 for controlling
//                a ROSRev2-class robot.
// Derived from RosRomeoNode by Mark Rose
//
// Joe Koning < koning at pobox dot com >

// Enable Interrupt libarary: https://github.com/GreyGnome/EnableInterrupt
// Download: https://bintray.com/greygnome/generic/EnableInterrupt/view#files
// No attachInterrupt(...) calls are allowed with this Libaray.
#include <EnableInterrupt.h>

// Needed on Leonardo to force use of USB serial.
#define USE_USBCON

// Simple PID from Mark Rose's git repo
//https://github.com/merose/SimplePID.git
#include <SimplePID.h>

//Ros 
//rosserial_arduino
//http://wiki.ros.org/rosserial_arduino
#include <ros.h>

// Pololu Astar lib
#include <AStar32U4.h>

//Std Arduino
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

//const int ONBOARD_SWITCH_PIN = A7;
const int ONBOARD_LED_PIN    = 17;

// The pins for motor control on the Romeo BLE.
const int M1_DIRECTION = 12;
const int M1_SPEED     = 9;
const int M2_SPEED     = 10;
//const int M2_DIRECTION = PE2;
//https://forum.arduino.cc/index.php?topic=347171.0
//PORTE |= 0x04;  // Set PE2
//PORTE &= 0xFB;  // Clear PE2
//PINE = 0x04;  // Toggle PE2
//val = !!(PINE & 0x04);  // digitalRead PE2
//DDRE |= 0x04;  // pinMode PE2 OUTPUT
//DDRE &= 0xFB;  // pinMode PE2 INPUT

// Pins for the Pololu motor encoder outputs. Using Pin Change Inerrupts
const int M1_A = 15;
const int M1_B = 16;
const int M2_A = 8;
const int M2_B = 11;

ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelVelocityMsg;
ros::Publisher lwheelVelocityPub("lwheel_velocity", &lwheelVelocityMsg);

std_msgs::Float32 rwheelVelocityMsg;
ros::Publisher rwheelVelocityPub("rwheel_velocity", &rwheelVelocityMsg);

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("lwheel_vtarget", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("rwheel_vtarget", &rwheelTargetCallback);

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
// Ku and Tu were determined by setting Ki and Kd to zero, then increasing
// Kp until steady oscillation occurs. Tu is the oscillation wavelength.
const float Ku = .19;
const float Tu = .23;
const float Kp = 0.45*Ku;
const float Ki = 1.2*Kp/Ku;
const float Kd = Kp*Tu/8;

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

// The number of encoder ticks per meter.
int ticksPerMeter;

// The number of milliseconds without a velocity target when the robot
// will automatically turn off the motors.
int vtargetTimeoutMillis;

unsigned long lastLoopTime;
unsigned long lastMotorCmdTime;

int leftMotorCmd = 0;
int rightMotorCmd = 0;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 60;

// Charge!
AStar32U4Buzzer buzzer;
const char mcharge[] PROGMEM =
  "! O5 L16 g8c8e8g8e16g2";
  
void setup()
{
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(M1_DIRECTION, OUTPUT);
  DDRE &= 0xFB;  //pinMode(M2_DIRECTION, OUTPUT);

  enableInterrupt(M1_A, leftAChange, CHANGE);
  enableInterrupt(M2_A, rightAChange,CHANGE);
  
  nh.initNode();

  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);

  // Wait until the node has initialized before getting parameters.
  while(!nh.connected()) {
    nh.spinOnce();
  }

  int controlRate;
  if (!nh.getParam("~control_rate", &controlRate)) {
    controlRate = 50;
  }
  controlDelayMillis = 1000.0 / controlRate;
  
  if (!nh.getParam("ticks_meter", &ticksPerMeter)) {
    ticksPerMeter = 50;
  }
  
  float vtargetTimeout;
  if (!nh.getParam("~vtarget_timeout", &vtargetTimeout)) {
    vtargetTimeout = 2.0;
  }
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;
  
  // Ready
  buzzer.playFromProgramSpace(mcharge);
}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  delay(controlDelayMillis);

  long curLoopTime = micros();

  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  lwheelMsg.data = (int) curLwheel;
  rwheelMsg.data = (int) curRwheel;
  lwheelPub.publish(&lwheelMsg);
  rwheelPub.publish(&rwheelMsg);

  float dt = (curLoopTime - lastLoopTime) / 1E6;

  float lwheelRate = ((curLwheel - lastLwheel) / dt);
  float rwheelRate = ((curRwheel - lastRwheel) / dt);

  lwheelVelocityMsg.data = lwheelRate / ticksPerMeter;
  rwheelVelocityMsg.data = rwheelRate / ticksPerMeter;
  lwheelVelocityPub.publish(&lwheelVelocityMsg);
  rwheelVelocityPub.publish(&rwheelVelocityMsg);
  
  int leftControl = leftController.getControlValue(lwheelRate, dt);
  leftMotorCmd += min(255, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -255, 255);
  if (leftMotorCmd > 0) {
    leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
  }
  
  int rightControl = rightController.getControlValue(rwheelRate, dt);
  rightMotorCmd += min(255, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -255, 255);
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

void rightAChange() {
  if (digitalRead(M2_A) != digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

// Sets the left and right motor speeds.
void setSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(M1_DIRECTION, (leftSpeed >= 0 ? HIGH : LOW));
  analogWrite(M1_SPEED, abs(leftSpeed));
  rightSpeed >= 0 ? (PORTE |= 0x04) : (PORTE &= 0xFB);//digitalWrite(M2_DIRECTION, (rightSpeed >= 0 ? HIGH : LOW));
  analogWrite(M2_SPEED, abs(rightSpeed));
}


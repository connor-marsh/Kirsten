#include "vex.h"
#include "iostream"

using namespace std;
using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
// Brain screen is 480 wide and 240 tall
double brainHeight = 240.0;
double brainWidth = 480.0;
int exitBWidth = 100;
int exitBHeight = 100;
int frameCount = 0;
// A global instance of vex::competition

// Motors

motor        LeftMotor1(PORT3, gearSetting::ratio18_1, false);
motor        RightMotor1(PORT1, gearSetting::ratio18_1, true);
motor        LeftMotor2(PORT4, gearSetting::ratio18_1, true);
motor        RightMotor2(PORT2, gearSetting::ratio18_1, false);

motor        ArmMotor(PORT11, gearSetting::ratio36_1, false);

motor        LeftTrackMotor(PORT12, gearSetting::ratio36_1, true);
motor        RightTrackMotor(PORT13, gearSetting::ratio36_1, false);

motor        SlapMotor(PORT5, gearSetting::ratio36_1, true);

double map(double val, double min1, double max1, double min2, double max2) {
  return (val-min1)*(max2-min2)/(max1-min1)+min2; 
}

// Controllers

controller   Driver(controllerType::primary);
controller   Operator(controllerType::partner);

// Sensors
// vex:: Inertial(PORT20);
pot          ArmPot(Brain.ThreeWirePort.H);
// ENCODER VALUES
// (Higher numbers are lower to the ground)
int armHeight = 0;
int minHeight = 199;
int maxHeight = 120;

double driveEncoder = 0;
double inchesPerDriveSensors = 1.489;
double gyroAngle = 0;
double degreesPerDriveSensors = 1.0 / 4.235;

double pctHeight(double pct) {
  // cout << "initial: " << pct<<"\n";
  // cout << "fina': " << map(pct, 0, 100, minHeight, maxHeight)<< "\n";
  return map(pct, 0, 100, minHeight, maxHeight);
}

double currentHeight() {
  return map(ArmPot.value(rotationUnits::deg), minHeight, maxHeight, 0, 100);
}

// 0 is nothing    1 is teleop     2 is auto    3 is both
int toDo = 3;

int controlMode = 3;
bool arcade = true;

// 1 is start with preload red   2 is preload blue
int autoRoutine = 0;
string screen = "Auto Select";
bool robotInitialized = false;

competition Competition;



#include "globals.h"
#include "iostream"
#include "thread"

using namespace std;
using namespace vex;

using namespace std;
using namespace vex;



// height is actually the degree of the arm pot
// {P, I, D, PREV, ACCUM, FEEDFORWARD, HITS, RUNNING_PID}
double armGains[] = {4, 0, 0.1, 0, 0, -1, 0, false};
double targetHeight = 0;
double driveGains[] = {4.5, 0, 0, 0, 0, 1, 0, false};
double driveForDistanceGains[] = {4.5, 0, 0, 0, 0, 1, 0, false};
double driveForAngleGains[] = {3.5, 0, 0, 0, 0, 1, 0, false};
double targetDrive = 0;
double driveScale = 1;
bool doingAngle = false;

double pid(double target, double process, double gains[]) {
  
  double error = target - process;

  return error * gains[0] + gains[4] * gains[1] + (error - gains[3]) * gains[2];

}

// ARM

int goToHeightController() {
  
  double height = targetHeight;
  cout << "Height: " << height << "\n";
  
  armGains[7] = true;

  while (armGains[6] < 4 && armGains[7]) {
  
    double output = pid(height, ArmPot.value(rotationUnits::deg), armGains);

    cout << "Output: " << output << "\n";

    armGains[4] += output;
    armGains[3] = output;

    if (output > 0 && output < 2) {
      armGains[6]++;
    }
    else if (output > -2 && output < 0) {
      armGains[6]++;
    }
    ArmMotor.spin(directionType::fwd, output*armGains[5], velocityUnits::pct);

    this_thread::sleep_for(20);

  }

  cout << "ARM DONE!" << "\n";
  cout << "ARM DONE!" << "\n";
  cout << "ARM DONE!" << "\n";
  cout << "ARM DONE!" << "\n";
  cout << "ARM DONE!" << "\n";

  armGains[6] = 0;
  armGains[7] = false;

  ArmMotor.spin(directionType::fwd, 0, velocityUnits::pct);

  return 1;

}

// async defaults to true
void goToHeight(double height, bool async) {

  targetHeight = height;
  if (async) {
    thread armThread(goToHeightController);
  }
  else {
    goToHeightController();
  }

}
void goToHeight(double height) {

  targetHeight = height;
  
  thread armThread(goToHeightController);

}

// DRIVE

int driveController() {

  driveGains[7] = true;

  if (doingAngle) {
    for (int i = 0; i < 7; i++) {
      driveGains[i] = driveForAngleGains[i];
    }
  }
  else {
    for (int i = 0; i < 7; i++) {
      driveGains[i] = driveForDistanceGains[i];
    }
  }

  this_thread::sleep_for(40);
  
  double dist = targetDrive;
  cout << "Calling drive for distance" << "\n";
  cout << "Distance: " << dist << "\n";
  cout << "Current values " << driveEncoder << "\n";

  while (driveGains[6] < 4 && driveGains[7]) {
  
    double output = pid(dist, (doingAngle)?gyroAngle:driveEncoder, driveGains);
    output *= driveScale;

    cout << "Output: " << output << "\n";

    driveGains[4] += output;
    driveGains[3] = output;

    if (output > 0 && output < 2) {
      driveGains[6]++;
    }
    else if (output > -2 && output < 0) {
      driveGains[6]++;
    }
    LeftMotor1.spin(directionType::fwd, output*driveGains[5]*((doingAngle)?-1:1), velocityUnits::pct);
    LeftMotor2.spin(directionType::fwd, output*driveGains[5]*((doingAngle)?-1:1), velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, output*driveGains[5], velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, output*driveGains[5], velocityUnits::pct);

    this_thread::sleep_for(20);

  }

  cout << "DRIVE DONE!" << "\n";
  cout << "DRIVE DONE!" << "\n";
  cout << "DRIVE DONE!" << "\n";
  cout << "DRIVE DONE!" << "\n";
  cout << "DRIVE DONE!" << "\n";

  driveGains[6] = 0;
  driveGains[7] = false;

  // LeftMotor1.spin(directionType::fwd, 0, velocityUnits::pct);
  // LeftMotor2.spin(directionType::fwd, 0, velocityUnits::pct);
  // RightMotor1.spin(directionType::fwd, 0, velocityUnits::pct);
  // RightMotor2.spin(directionType::fwd, 0, velocityUnits::pct);

  LeftMotor1.stop();
  LeftMotor2.stop();
  RightMotor1.stop();
  RightMotor2.stop();

  return 1;

}

// async defaults to true
void driveForDistance(double dist, bool async) {

  targetDrive = dist;
  driveEncoder = 0;
  doingAngle = false;
  driveScale = 1;
  if (async) {
    thread driveThread(driveController);
  }
  else {
    driveController();
  }

}
void driveForDistance(double dist) {

  targetDrive = dist;
  driveEncoder = 0;
  doingAngle = false;
  driveScale = 1;
  
  thread driveThread(driveController);

}
void driveForDistance(double dist, double pctSpeed) {

  targetDrive = dist;
  driveEncoder = 0;
  doingAngle = false;
  driveScale = pctSpeed;
  
  thread driveThread(driveController);

}

// async defaults to true
// turnToAngle is absolute not relative angle (0 is facing right, 270 is up)
void turnToAngle(double ang, bool async) {

  targetDrive = ang;
  doingAngle = true;
  gyroAngle = 0;
  if (async) {
    thread driveThread(driveController);
  }
  else {
    driveController();
  }

}
void turnToAngle(double ang) {

  targetDrive = ang;
  doingAngle = true;
  gyroAngle = 0;
  
  thread driveThread(driveController);

}

void waitForDriveCommand() {
  this_thread::sleep_for(20);
  while (driveGains[7]) {
    this_thread::sleep_for(50);
  }
}

void dump(bool outtake) {

  if (outtake) {
    RightTrackMotor.startRotateFor(directionType::rev, 0.25, rotationUnits::rev, 22, velocityUnits::pct);
    LeftTrackMotor.rotateFor(directionType::rev, 0.25, rotationUnits::rev, 22, velocityUnits::pct); 
    this_thread::sleep_for(100);
  }

  SlapMotor.rotateFor(directionType::fwd, 60, rotationUnits::deg, 50, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(300);
  SlapMotor.rotateFor(directionType::fwd, 60, rotationUnits::deg, 20, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(200);
  SlapMotor.rotateFor(directionType::fwd, 50, rotationUnits::deg, 10, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(directionType::rev, 0.9, rotationUnits::rev, 50, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(directionType::rev, 0.9, rotationUnits::rev, 50, velocityUnits::pct);
  driveForDistance(-30);
  waitForDriveCommand();
  

}
void dump() {

  SlapMotor.rotateFor(directionType::fwd, 60, rotationUnits::deg, 50, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(300);
  SlapMotor.rotateFor(directionType::fwd, 60, rotationUnits::deg, 20, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(200);
  SlapMotor.rotateFor(directionType::fwd, 50, rotationUnits::deg, 10, velocityUnits::pct);
  SlapMotor.stop();
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(directionType::rev, 0.9, rotationUnits::rev, 50, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(directionType::rev, 0.9, rotationUnits::rev, 50, velocityUnits::pct);
  driveForDistance(-30);
  waitForDriveCommand();
  
  

}
// 268.065
// 268.065 / (15ft*12in) = 1.489
void updateSensors() {
  while (true) {
    double left1 = LeftMotor1.velocity(percentUnits::pct);
    double left2 = LeftMotor2.velocity(percentUnits::pct);
    double right1 = RightMotor1.velocity(percentUnits::pct);
    double right2 = RightMotor2.velocity(percentUnits::pct);

    double totalVel = ((left1 + left2)/2 + (right1 + right2)/2) / 2 / 40;
    
    driveEncoder += totalVel / inchesPerDriveSensors;
    
    double turningVel = ((left1 + left2) / -2.0 + (right1 + right2) / 2.0) / 2.0 / 40.0;

    gyroAngle += turningVel / degreesPerDriveSensors;

    cout << "Angle: " << gyroAngle << "\n";
    
    //cout << "Drive Encoders: " << driveEncoder << "\n";
    this_thread::sleep_for(40);

  }
}

void initRobot() {
  // Shake slap
  SlapMotor.rotateFor(200, timeUnits::msec, 100, velocityUnits::pct);
  SlapMotor.rotateFor(200, timeUnits::msec, -100, velocityUnits::pct);
  // Raise slap and arm
  SlapMotor.startRotateFor(directionType::fwd, 200, rotationUnits::deg, 60, velocityUnits::pct);
  ArmMotor.rotateFor(directionType::fwd, 350, rotationUnits::deg, 100, velocityUnits::pct);
  // Lower arm
  //ArmMotor.rotateFor(directionType::rev, 50, rotationUnits::deg, 100, velocityUnits::pct);
  ArmMotor.startRotateFor(directionType::rev, 230, rotationUnits::deg, 100, velocityUnits::pct);

  // Put slap back
  SlapMotor.startRotateFor(directionType::rev, 200, rotationUnits::deg, 60, velocityUnits::pct);
  //ArmMotor.rotateFor(directionType::fwd, -60, rotationUnits::deg);
  robotInitialized = true;
}



void redDumpSimple() {
  // Get cubes in a line
  
  driveForDistance(50, 0.5);
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);

  // Go back and turn
  cout << "Drive back" << "\n";
  driveForDistance(-26);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  
  // Go to goal and dump
  turnToAngle(-144);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  driveForDistance(30);
  RightTrackMotor.startRotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();
  dump(true);
  
}

// void redDump() {
//   cout << "WOW" << "\n";
//   // Get cubes in a line
//   driveForDistance(59, 0.5);
//   this_thread::sleep_for(200);
//   RightTrackMotor.startRotateFor(2.8, rotationUnits::rev);
//   LeftTrackMotor.rotateFor(2.8, rotationUnits::rev);


//   // Go back and turn
//   cout << "Drive back" << "\n";
//   driveForDistance(-32);
//   waitForDriveCommand();
//   //turnToAngle(270);
//   // Get the 1 extra cube
//   cout << "Get cube" << "\n";
//   driveForDistance(26);
//   this_thread::sleep_for(100);
//   RightTrackMotor.startRotateFor(0.5, rotationUnits::rev);
//   LeftTrackMotor.rotateFor(0.5, rotationUnits::rev);
//   waitForDriveCommand();
//   // Go to goal and dump
//   //turnToAngle(235);
//   driveForDistance(15);
//   waitForDriveCommand();
//   dump(true);
//   waitForDriveCommand();
// }

void blueDump() {
  // Get cubes in a line
  
  driveForDistance(50, 0.5);
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);

  // Go back and turn
  cout << "Drive back" << "\n";
  driveForDistance(-26);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  
  // Go to goal and dump
  turnToAngle(144);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  driveForDistance(34);
  RightTrackMotor.startRotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();
  dump(true);
}

void redTower() {
  // Get cubes in a line
  
  driveForDistance(36, 0.4);
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);

  // Go back and turn
  cout << "Drive back" << "\n";
  driveForDistance(-12);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  
  // Go to goal and dump
  turnToAngle(-144);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  driveForDistance(30);
  RightTrackMotor.startRotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();
  dump(true);
}

void blueTower() {
  // Get cubes in a line
  
  driveForDistance(20, 0.5);
  this_thread::sleep_for(200);
  RightTrackMotor.startRotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(2.8, rotationUnits::rev, 70, velocityUnits::pct);

  // Go back and turn
  cout << "Drive back" << "\n";
  driveForDistance(-10);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  
  // Go to goal and dump
  turnToAngle(136);
  RightTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.startRotateFor(1, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();

  driveForDistance(30);
  RightTrackMotor.startRotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  LeftTrackMotor.rotateFor(1.4, rotationUnits::rev, 70, velocityUnits::pct);
  waitForDriveCommand();
  dump(true);
}

void autonomous() {

  if (!robotInitialized) {
    initRobot();
  }
  
  switch (autoRoutine) {
    case 0:
      redDumpSimple();
      // redDump();
      break;
    case 1:
      blueDump();
      break;
    case 2:
      redTower();
      break;
    case 3:
      blueTower();
  }

}
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "iostream"
#include "auto.h"
#include "future"

using namespace vex;
using namespace std;


void draw() {
  Brain.Screen.clearScreen();
  if (screen == "Auto Select") {
    Brain.Screen.setFillColor(0);
    Brain.Screen.drawRectangle(0, 0, brainWidth/2.0, brainHeight/2.0);
    Brain.Screen.drawRectangle(0, brainHeight/2.0, brainWidth/2.0, brainHeight/2.0);
    Brain.Screen.setFillColor(240);
    Brain.Screen.drawRectangle(brainWidth/2.0, 0, brainWidth/2.0, brainHeight/2.0);
    Brain.Screen.drawRectangle(brainWidth/2.0, brainHeight/2.0, brainWidth/2.0, brainHeight/2.0);
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::black);
    Brain.Screen.printAt(10, 20, false, "Red Dump");
    Brain.Screen.printAt(brainWidth - (brainWidth/2.0), 20, false, "Blue Dump");
    Brain.Screen.printAt(brainWidth/2.0-100, brainHeight-10, false, "Red Tower");
    Brain.Screen.printAt(brainWidth/2.0+10, brainHeight-10, false, "Blue Tower");
  }
  else if (screen == "Auto Selected") {
    Brain.Screen.setFillColor(100);
    Brain.Screen.drawRectangle(0, 0, brainWidth, brainHeight);
    char* routineName;
    if (autoRoutine == 0) {
      routineName = "Auto Routine: Red (Dump)";
    }
    else if (autoRoutine == 1) {
      routineName = "Auto Routine: Blue (Dump)";
    }
    else if (autoRoutine == 2) {
      routineName = "Auto Routine: Red (Tower)";
    }
    else if (autoRoutine == 3) {
      routineName = "Auto Routine: Blue (Tower)";
    }
    else {
      routineName = "nonreal";
    }
    Brain.Screen.setPenColor(color::black);
    Brain.Screen.printAt(brainWidth/4.0, brainHeight/2.0, false, routineName);
    Brain.Screen.setFillColor(0);
    Brain.Screen.drawRectangle(brainWidth-exitBWidth, brainHeight-exitBHeight, exitBWidth, exitBHeight);
    Brain.Screen.printAt(brainWidth-exitBWidth+exitBWidth/10, brainHeight-exitBHeight/2, false, "exit");

  }
}
void pressDetect() {
  int x = Brain.Screen.xPosition();
  int y = Brain.Screen.yPosition();

  if (screen == "Auto Select") {

    if (x >= 0 && x <= brainWidth/2.0 && y >= 0 && y <= brainHeight/2.0) {
      autoRoutine = 0;
      screen = "Auto Selected";
    }
    else if (x >= brainWidth/2.0 && x <= brainWidth && y >= 0 && y <= brainHeight/2.0) {
      autoRoutine = 1;
      screen = "Auto Selected";
    }
    else if (x >= 0 && x <= brainWidth/2.0 && y >= brainHeight/2.0 && y <= brainHeight) {
      autoRoutine = 2;
      screen = "Auto Selected";
    }
    else if (x >= brainWidth/2.0 && x <= brainWidth && y >= brainHeight/2.0 && y <= brainHeight) {
      autoRoutine = 3;
      screen = "Auto Selected";
    }

  }

  else if (screen == "Auto Selected") {

    if (x >= brainWidth-exitBWidth && x <= brainWidth && y >= brainHeight-exitBHeight && y <= brainHeight) {
      autoRoutine = 1;
      screen = "Auto Select";
    }

  }

}

/*
**********PRE-AUTON*******************PRE-AUTON**********
**********PRE-AUTON*******************PRE-AUTON**********
**********PRE-AUTON*******************PRE-AUTON**********
*/

void pre_auton() {
  draw();
  Brain.Screen.pressed(pressDetect);
  LeftMotor1.resetRotation();
  LeftMotor2.resetRotation();
  RightMotor1.resetRotation();
  RightMotor2.resetRotation();
  ArmMotor.resetRotation();
  LeftTrackMotor.resetRotation();
  RightTrackMotor.resetRotation();
  SlapMotor.resetRotation();
  ArmMotor.setStopping(hold);
  SlapMotor.setStopping(hold);
  LeftMotor1.setStopping(hold);
  LeftMotor2.setStopping(hold);
  RightMotor1.setStopping(hold);
  RightMotor2.setStopping(hold);


}

double abs(double num) {
  if (num > 0) {
    return num;
  }
  else {
    return -1*num;
  }
}
int abs(int num) {
  if (num > 0) {
    return num;
  }
  else {
    return -1*num;
  }
}

/*

************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************
************TELEOP************

*/

void teleop() {
  frameCount = 0;
  if (!robotInitialized) {
    initRobot();
  }

  driveGains[7] = false;

  while(1) {
    frameCount++;
    //cout << ArmPot.value(rotationUnits::deg) << "\n";
    
    // Brain.Screen.printAt( 10, 50, "Hello V5 %d", count++ );
    // Allow other tasks to run
    // int val = Gyro.value(percentUnits::pct);
    //cout << val;
    int armPower = 0;
    double trackPower = 0;
    double slapPower = 0;
    double leftPower;
    double rightPower;
    if (arcade) {
      leftPower = Driver.Axis3.value() + Driver.Axis1.value();
      rightPower = Driver.Axis3.value() + -Driver.Axis1.value();
      if (leftPower > 127) {
        leftPower = 127;
      }
      if (leftPower < -127) {
        leftPower = -127;
      }
      if (rightPower > 127) {
        rightPower = 127;
      }
      if (rightPower < -127) {
        rightPower = -127;
      }
    }
    else {
      leftPower = Driver.Axis3.value()*0.8;
      rightPower = Driver.Axis2.value()*0.8;
    }

    if (abs(leftPower) < 10) {
      leftPower = 0;
    }
    if (abs(rightPower) < 10) {
      rightPower = 0;
    }

    if (Driver.ButtonRight.pressing() && controlMode == 2) {
      controlMode = 1;
    }
    if (Driver.ButtonLeft.pressing() && controlMode == 1) {
      controlMode = 2;
    }

    if (controlMode == 1) {

      if (Driver.ButtonR2.pressing()) {
        rightPower *= 0.5;
        leftPower *= 0.5;
      }
      else if (Driver.ButtonL2.pressing()) {
        rightPower *= 0.2;
        leftPower *= 0.2;
      }

      bool rT = Driver.ButtonR1.pressing();
      bool lT = Driver.ButtonL1.pressing();

      //bool a = Driver.ButtonA.pressing();
      bool b = Driver.ButtonB.pressing();
      bool x = Driver.ButtonX.pressing();
      //bool y = Driver.ButtonY.pressing();

    

      //bool left = Driver.ButtonLeft.pressing();
      bool down = Driver.ButtonDown.pressing();
      bool up = Driver.ButtonUp.pressing();
      //bool right = Driver.ButtonRight.pressing();
    
      
      // Get arm power

      if (x) {
        armPower = 65;
      }
      else if (b) {
        armPower = -65;
      }
      else {
        armPower = 0;
      }

      // Get track power

      if (rT && lT) {
        trackPower = 100;
      }
      else if (rT) {
        trackPower = 50;
      }
      else if (lT) {
        trackPower = -40;
      }
      else {
        trackPower = 0;
      }

      // Get slap power

      if (up) {
        slapPower = 50;
      }
      else if (down) {
        slapPower = -30;
      }
      else {
        slapPower = 0;
      }

    }

    else {

      armPower = Operator.Axis2.value();
      slapPower = Operator.Axis3.value();

      if (Operator.ButtonDown.pressing()) {
        slapPower *= 0.3;
      }

      // Slow Moooode!
      if (Driver.ButtonR1.pressing()) {
        leftPower *= 0.5;
        rightPower *= 0.5;
      }
      else if (Driver.ButtonR2.pressing()) {
        leftPower *= 0.35;
        rightPower *= 0.35;
      }
      else if (Driver.ButtonL1.pressing() || Driver.ButtonL2.pressing()) {
        leftPower *= 0.15;
        rightPower *= 0.15;
      }

      bool slowIntake = Operator.ButtonR1.pressing();
      bool slowOuttake = Operator.ButtonL1.pressing();
      bool fastOuttake = Operator.ButtonL2.pressing();
      bool fastIntake = Operator.ButtonR2.pressing();

      if (fastOuttake) {
        trackPower = -100;
      }
      else if (fastIntake) {
        trackPower = 100;
      }
      else if (slowIntake) {
        trackPower = 50;
      }
      else if (slowOuttake) {
        trackPower = -40;
      }
      else {
        trackPower = 0;
      }

    

    }
    // If not running go to height
    if (!armGains[7]) {

      // Call height presets
      if (Operator.ButtonY.pressing()) {
        goToHeight(pctHeight(0));
      }
      else if (Operator.ButtonA.pressing()) {
        goToHeight(pctHeight(60));
      }
      else if (Operator.ButtonX.pressing()) {
        goToHeight(pctHeight(80));
      }

      if (ArmPot.value(rotationUnits::deg) < pctHeight(100) && armPower > 1) {
        ArmMotor.stop();
      }
      // else if (ArmPot.value(rotationUnits::deg) > pctHeight(0) && armPower < -1) {
      //   ArmMotor.stop();
      // }
      else if (ArmMotor.isSpinning() == 0) {
        if (abs(armPower) < 10) {
          ArmMotor.stop();

        }
        else {
          ArmMotor.spin(vex::directionType::fwd, armPower, vex::velocityUnits::pct);
        }
      }
    }
    else {
      // Stop go to height
      if (Operator.ButtonB.pressing()) {
        armGains[7] = false;
      }
    }

    if (abs(slapPower) > 10) {
      SlapMotor.spin(vex::directionType::fwd, slapPower, vex::velocityUnits::pct);
    }
    else {
      SlapMotor.stop();
    }
  

    LeftTrackMotor.spin(vex::directionType::fwd, trackPower, vex::velocityUnits::pct);
    RightTrackMotor.spin(vex::directionType::fwd, trackPower, vex::velocityUnits::pct);

    RightMotor1.spin(vex::directionType::fwd, rightPower, vex::velocityUnits::pct);
    RightMotor2.spin(vex::directionType::fwd, rightPower, vex::velocityUnits::pct);

    LeftMotor1.spin(vex::directionType::fwd, leftPower, vex::velocityUnits::pct);
    LeftMotor2.spin(vex::directionType::fwd, leftPower, vex::velocityUnits::pct);
        
    this_thread::sleep_for(20);

    }

}






//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( teleop );
    
    //Run the pre-autonomous function.
    pre_auton();
    thread sensorThread(updateSensors);
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      draw();
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
}
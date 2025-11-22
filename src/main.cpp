/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/10/2025, 4:53:21 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
// A global instance of competition
competition Competition;

// auton toggles: 
// 0 = Left Long Goal reg
// 1 = Right Long Goal with descore
// 2 = WP Right
// 3 = WP Left
// 4 = skills goal 1
// 5 = skills goal 2
// 6 = skills goal 3
// 7 = skills park
int autonToggle = 4;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;
inertial Gyro = inertial(PORT19);
//drivebase
motor leftFront = motor(PORT5, ratio6_1, false);
motor leftMiddle = motor(PORT6, ratio6_1, false);
motor leftBack = motor(PORT13, ratio6_1, false);
motor rightFront = motor(PORT11, ratio6_1, true);
motor rightMiddle = motor(PORT1, ratio6_1, true);
motor rightBack = motor(PORT2, ratio6_1, true);
motor_group leftSide = motor_group(leftFront, leftMiddle, leftBack);
motor_group rightSide = motor_group(rightFront, rightMiddle, rightBack);

//intakes
motor rollersBottom = motor(PORT12, ratio6_1, true); // rollersBottom
motor rollersTop = motor(PORT17, ratio18_1, false); // rollersTop
motor topIntake = motor(PORT3, ratio18_1, false);
  
//pneumatics
digital_out descore = digital_out(Brain.ThreeWirePort.G);
digital_out matchLoader = digital_out(Brain.ThreeWirePort.H);
digital_out stopPiston = digital_out(Brain.ThreeWirePort.F); // false = open true = close
//digital_out frontDescore = digital_out(Brain.ThreeWirePort.F);

// constants
double pi = 3.1415926;
double diameter = 3.25;
double g = 36.0/48.0;
double width = 13.75;

// piston controls
void loaderControl() {
  matchLoader.set(!matchLoader.value());
}

void descoreControl() {
  descore.set(!descore.value());
}

void unloading() {
  stopPiston.set(!stopPiston.value());
}

// stop all rollers
void stopAll() {
  rollersBottom.stop();
  rollersTop.stop();
  topIntake.stop();
}

// stop top rollers
void stopTop() {
  rollersTop.stop();
  topIntake.stop();
}

void drive(int lspeed, int rspeed, int wt) {
  leftSide.spin(forward, lspeed, pct);
  rightSide.spin(forward, rspeed, pct);
  wait(wt, msec);
}

//auton functions
// intake to the top goal
void intakeTop() {
  rollersBottom.spin(reverse, 100, pct);
  rollersTop.spin(forward, 80, pct);
  topIntake.spin(forward, 100, pct);
}
// intake just the bottom intake
void intakeBottom() {
  rollersBottom.spin(reverse, 100, pct);
}
// intake to the middle top goal
void intakeMiddleTop() {
  rollersBottom.spin(reverse, 100, pct);
  rollersTop.spin(forward, 100, pct);
  topIntake.spin(reverse, 70, pct);
}
// intake to the middle bottom goal
void intakeMiddleBottom() {
  rollersBottom.spin(forward, 30, pct);
  rollersTop.spin(reverse, 100, pct);
}
// stop all intakes
void intakeStop() {
  rollersBottom.stop();
  rollersTop.stop();
  topIntake.stop();
}

void DriveVolts(double lspeed, double rspeed, double multipier, int wt) {
  lspeed = lspeed * 120 * multipier;
  rspeed = rspeed * 120 * multipier;
  leftFront.spin(forward, lspeed, voltageUnits::mV);
  leftMiddle.spin(forward, lspeed, voltageUnits::mV);
  leftBack.spin(forward, lspeed, voltageUnits::mV);
  rightFront.spin(forward, rspeed, voltageUnits::mV);
  rightMiddle.spin(forward, rspeed, voltageUnits::mV);
  rightBack.spin(forward, rspeed, voltageUnits::mV);
}

void inchDrive(float target, int timeout = 1200, float kp = 3.8) {
  timer t2;
  t2.reset();

  float x = 0.0;
  float ap = 2.5; // increase = more wiggle // decrease = less wiggle 3
  float aerror = 0;
  float Atarget = Gyro.rotation();
  float heading = Gyro.rotation();
  float aspeed = 0.0;
  const float tolerance = 1.0;
  const float accuracy = 1.0; //may need adjust
  int count = 0;

  const float ki = 0.0;
  const float kd = 2.65;
 
  float integral = 0;
  float prevError = target;
  float derivative = 0;
  
  float error = target - x;
  float speed = error * kp;
 
  leftFront.setPosition(0.0, rev);
  rightFront.setPosition(0.0, rev);
  
  while (t2.time(msec) < timeout) {
    heading = Gyro.rotation();
    aerror = Atarget - heading;
    aspeed = ap*aerror;
  
    x = ((rightFront.position(rev) + leftFront.position(rev)) / 2.0) * pi * diameter * g;
    error = target - x;
    if (fabs(error) < accuracy)
    {
      count++;
    }
    else {
      count = 0;
    }
    
    if (count > 20) {
      break;
    }
  
    if (fabs(error) < tolerance) {
      integral += error;
    }
    
    speed = error * kp + integral * ki + derivative * kd;
    
    if(speed >= 100){
      speed = 100;
    }
    
    if (speed <= -100){
      speed = -100;
    }
    
    derivative = error - prevError;
    prevError = error;
    DriveVolts(speed+aspeed, speed-aspeed, 1, 10);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.clearLine();
    Controller1.Screen.print(error);
    wait(10, msec);
  }
  leftSide.stop(brake);
  rightSide.stop(brake);
  wait(10, msec);
}

void gyroturnAbs(double target, int timeout = 1200) {
    timer t1;
    t1.reset();
    float kp = 1.75;
    float ki = 0;
    float kd = 0.6;
    float integral = 0;
    float integralTolerance = 3;
    // float integralMax = 100;
    float heading = 0.0;
    float error = target - heading;
    float prevError = 0;
    float derivative;
    float speed = kp * error;
    float accuracy = 0.1;
    float bias = 0;
    int count = 0;
    while (t1.time(msec) < timeout) {
      heading = Gyro.rotation(degrees);
      error = target - heading;
      derivative = (error - prevError);
      prevError = error;
      if (fabs(error) < integralTolerance)
      {
        integral += error;
      }
      if (fabs(error) < accuracy)
      {
        count++;
      }
      else {
        count = 0;
      }
      if (count > 20) {
        break;
      }
      speed = kp * error + kd * derivative + ki * integral;
      DriveVolts(speed, -speed, 1, 0);
      wait(10, msec);
    }
    leftSide.setStopping(brake);
    leftSide.stop();
    rightSide.setStopping(brake);
    rightSide.stop();
    wait(10, msec);
  }

// rd = radius of circular path
void arcTurn(float rd, float angle, float maxSpeed = 100) {
    float kp = 12.0;
    float kd = 1.0;
    float targetArcLength = rd * 2 * pi * (angle/360.0);
    float arcLength = 0.0;
    float error = targetArcLength - arcLength;
    float oldError = error;
    float lspeed = (maxSpeed * angle) / fabs(angle);
    float rspeed = (lspeed * (rd - width)) / rd;
    float accuracy = 0.2;
    leftMiddle.setPosition(0.0, rev);
    rightMiddle.setPosition(0.0, rev);
    while (fabs(error) >= accuracy) {
      DriveVolts(lspeed, rspeed, 1, 10);
      arcLength = leftMiddle.position(rev) * g * pi * diameter;
      oldError = error;
      error = targetArcLength - arcLength;
      lspeed = (kp * error) + (kd * (error-oldError));
      if (fabs(lspeed) >= maxSpeed) {
        lspeed = (maxSpeed * error) / fabs(error);
        rspeed = (lspeed * (rd - width)) / rd;
      }
      // leftSide.setStopping(brake);
      // rightSide.setStopping(brake);
      // leftSide.stop();
      // rightSide.stop();
    }
}

void longGoalLeft() {
  intakeTop();
  stopPiston.set(false);
  inchDrive(30, 680);
  gyroturnAbs(-28, 700);
  inchDrive(7, 700); // getting blocks
  matchLoader.set(true);
  inchDrive(5, 500);
  gyroturnAbs(-120, 800); 
  inchDrive(35.5, 880); // driving to match loader
  matchLoader.set(true);
  gyroturnAbs(-175, 900); // 185
  inchDrive(18.5, 800, 3.5); // match loading
  wait(340, msec);
  stopAll();
  stopPiston.set(true);
  inchDrive(-32, 1200, 2.6);
  intakeTop();
  wait(3000, msec);
  inchDrive(10, 450);
  stopPiston.set(false);
  inchDrive(-19, 2000, 4);
}

void longGoalRight() {
  intakeTop();
  stopPiston.set(false);
  inchDrive(30, 680); // drive to 3 blocks
  gyroturnAbs(28, 390); // turn for 3 blocks
  inchDrive(7, 700); // getting blocks
  matchLoader.set(true);
  inchDrive(5, 500);
  gyroturnAbs(120, 720);
  inchDrive(36.5, 880); // goijg to goal // og: 33 // middle field: 33.8 // lessen if goal is more on the left
  matchLoader.set(true);
  gyroturnAbs(179, 770);
  inchDrive(18, 800, 3.2); // match loading
  wait(290, msec); // need to lessen this mayb
  stopTop();
  stopPiston.set(true);
  inchDrive(-32, 1200, 2.6);
  intakeTop();
  wait(2300, msec);
  intakeStop(); // done with goal
  inchDrive(10, 450);
  stopPiston.set(false);
  inchDrive(-12);
  leftSide.stop(hold);
  rightSide.stop(hold);
}

void WPRight() {
  intakeTop();
  stopPiston.set(false);
  inchDrive(30, 680); // drive to 3 blocks
  gyroturnAbs(28, 400); // turn for 3 blocks
  inchDrive(7, 500); // getting blocks
  matchLoader.set(true);
  inchDrive(5, 600);
  gyroturnAbs(56, 580);
  matchLoader.set(false);
  inchDrive(37, 650); // going under
  matchLoader.set(true);
  intakeBottom();
  wait(500, msec);
  inchDrive(-21, 670);
  matchLoader.set(false);
  rollersTop.spin(reverse, 100, pct);
  intakeMiddleBottom(); // scoring
  gyroturnAbs(-38, 690);
  inchDrive(17.5, 700);
  wait(630, msec);
  intakeTop();
  inchDrive(-48.5, 1100); // going to goal
  matchLoader.set(true);
  gyroturnAbs(-173, 1000);
  inchDrive(20, 800, 3.3); // match loading
  wait(380, msec);
  stopTop();
  stopPiston.set(true);
  inchDrive(-301, 1200, 2.6);
  intakeTop();
  wait(2050, msec);
  inchDrive(10, 500);
  stopPiston.set(false);
  inchDrive(-19, 2000, 4);
}

// NEED TO FINISH
void WPLeft() {
  intakeTop();
  stopPiston.set(false);
  inchDrive(30, 680); // drive to 3 blocks
  gyroturnAbs(-28, 400); // turn for 3 blocks
  inchDrive(7, 500); // getting blocks
  matchLoader.set(true);
  inchDrive(9, 600);
  rollersTop.spin(reverse, 100, pct);
  topIntake.spin(reverse, 100, pct);
  wait(300, msec);
  rollersTop.stop();
  topIntake.stop();
  gyroturnAbs(-130, 800);
  inchDrive(-16, 800);
  intakeMiddleTop();
  wait(900, msec);
  stopAll();
  inchDrive(45);
}

void skillsFirstGoal() {
  intakeTop();
  stopPiston.set(false);
  inchDrive(21, 620);
  gyroturnAbs(40, 400);
  inchDrive(20, 660);
  gyroturnAbs(125, 590); 
  inchDrive(28, 820); // drive to goal
  matchLoader.set(true);
  gyroturnAbs(182, 510); // turn to match load
  inchDrive(15.5, 1700, 3.5); // match loading
  wait(400, msec);
  inchDrive(-32.5, 1350, 2.6);
  stopPiston.set(true);
  intakeTop();
  wait(2500, msec);
  stopAll(); // end first goal
}

void skillsSecondGoal() {
  matchLoader.set(false);
  inchDrive(15, 540);
  gyroturnAbs(-80); // TIME
  inchDrive(-14, 640);
  gyroturnAbs(0, 690);
  descore.set(true);
  inchDrive(80, 1750, 3.0); // wall reset
  gyroturnAbs(-45, 690);
  inchDrive(17.3, 620); // drive to goal
  matchLoader.set(true);
  gyroturnAbs(0, 650);
  stopPiston.set(false);
  intakeTop();
  inchDrive(20, 2100, 3.5); // match loading
  inchDrive(-32, 1400, 2.6); // scoring
  stopPiston.set(true);
  intakeTop();
  wait(3000, msec);
  stopAll(); // end second goal
  matchLoader.set(false);
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  Gyro.calibrate();
  while (Gyro.isCalibrating()) {
    wait(20, msec);
  }
  wait(200, msec);
  Gyro.setHeading(0, deg);
  Gyro.setRotation(0, deg);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  switch(autonToggle) {
    case 0: // left side regular
      longGoalLeft();
      break;
    case 1: // right side doesnt work
      longGoalRight();
      break;
    case 2: 
      WPRight();
      break;
    case 3:
      WPLeft();
      break;
    case 4: // SKILLS 1-5
      skillsFirstGoal();
    case 5:
      skillsSecondGoal();
      break;
    case 6: // SKILLS 6-8
      inchDrive(15, 575);
      gyroturnAbs(-80, 655);
      inchDrive(118, 1950, 3); // wall reset
      inchDrive(-17.2, 600);
      gyroturnAbs(0, 900); // rigyt angle 340
      matchLoader.set(true);
      inchDrive(-24, 1000, 3.5); // driving backwards to goal
      //gyroturnAbs(-175, 90); // extra turning
      intakeTop();
      stopPiston.set(false);
      inchDrive(35, 3000, 3); // match loading
      gyroturnAbs(-8);
      inchDrive(-32, 1400, 2.7); // driving to goal
      stopPiston.set(true);
      intakeTop();
      wait(3000, msec);
      stopAll();
      matchLoader.set(false); // end third goal
    case 7: // parking from 3rd
      inchDrive(15, 590);
      gyroturnAbs(130, 850);
      intakeTop();
      stopPiston.set(false);
      inchDrive(23, 900);
      wait(400, msec);
      stopAll();
      rollersBottom.spin(reverse, 100, pct);
      gyroturnAbs(176, 800);
      inchDrive(97, 2100, 3.2);
      rollersBottom.stop();
      gyroturnAbs(87, 300);
      intakeTop();
      inchDrive(57, 1200, 6);
      break;
    case 8: // wip 4th goal
      inchDrive(15, 590);
      gyroturnAbs(130, 850);
      intakeTop();
      stopPiston.set(false);
      inchDrive(23, 900);
      wait(400, msec);
      stopAll();
      rollersBottom.spin(forward, 100, pct);
      gyroturnAbs(176, 800);
      inchDrive(66);
      rollersBottom.stop();
      gyroturnAbs(245, 1000); // turn to face goal
      inchDrive(25);
      matchLoader.set(true); // maybe add an aligner
      gyroturnAbs(183);
      intakeTop();
      inchDrive(30, 2000, 3.4);
      stopTop();
      break;
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Controller1.ButtonDown.pressed(loaderControl);
  Controller1.ButtonY.pressed(unloading);
  Controller1.ButtonB.pressed(descoreControl);
  //Controller1.Button
  while (1) {
    Brain.Screen.print("bottom voltage: ");
    Brain.Screen.print(rollersBottom.voltage(volt));
    Brain.Screen.newLine();
    Brain.Screen.print("top voltage: ");
    Brain.Screen.print(rollersTop.voltage(volt));
    Brain.Screen.newLine();
    double sensitivity = 0.7;
    int leftSpeed = (Controller1.Axis3.position(pct) + Controller1.Axis1.position(pct)) * sensitivity;
    int rightSpeed = (Controller1.Axis3.position(pct) - Controller1.Axis1.position(pct)) * sensitivity;
    drive (leftSpeed, rightSpeed, 10);
    // scoring top top
    if (Controller1.ButtonR1.pressing()) {
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(forward, 100, pct);
      if (Controller1.ButtonL1.pressing()) {
        rollersBottom.spin(reverse, 100, pct);
      }
      else {
        rollersBottom.stop();
      }
    }
    // scoring bottom top
    else if (Controller1.ButtonR2.pressing()) {
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(reverse, 70, pct);
    }
    // rotating the bottom rollers
    else if (Controller1.ButtonL1.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      // scoring on the top
      if (Controller1.ButtonR1.pressing()) {
        rollersTop.spin(forward, 80, pct);
        topIntake.spin(forward, 100, pct);
      } // scoring bottom top
      else if (Controller1.ButtonR2.pressing()) {
        rollersTop.spin(forward, 100, pct);
        topIntake.spin(reverse, 100, pct);        
      }
      else {
        rollersTop.stop();
        topIntake.stop();
      }
    }
    // remove balls out of robot
    else if (Controller1.ButtonL2.pressing()) {
      rollersTop.spin(reverse, 100, pct);
      rollersBottom.spin(forward, 100, pct);
      topIntake.spin(reverse, 100, pct);
    }
    else {
      rollersBottom.stop();
      topIntake.stop();
      rollersTop.stop();
    }
    /*
    if (Controller1.ButtonR1.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(forward, 100, pct);
    }
    // scoring bottom
    else if (Controller1.ButtonR2.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      topIntake.spin(reverse, 100, pct);
      rollersTop.spin(forward, 60, pct);
    }
    // descoring
    else if (Controller1.ButtonL1.pressing()) {
      rollersBottom.spin(forward, 100, pct);
      rollersTop.spin(reverse, 100, pct);
    }
    // loop to stop the rollers when there are more than 4 balls 
    if ((stopPiston.value() == true) && Controller1.ButtonR1.pressing()) {
      int count = 0;
      while (count <= 4) {
        if (// color == blue or color == red) {
          rollersBottom.spin(reverse, 100, pct);
          rollersTop.spin(forward, 100, pct);
          topIntake.spin(forward, 100, pct);    
          count++;  
        }
      }
      rollersTop.stop();
      topIntake.stop();
    }
    */
    wait(10, msec);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(10, msec); // 100
  }
}
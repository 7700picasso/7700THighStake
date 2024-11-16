/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       7700T                                                     */
/*    Created:      9/12/2024, 6:27:17 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;
float pi = 3.14;
float gr = 0.43;
float Dia = 4.0;

// define your global instances of motors and other devices here
motor LM = motor(PORT10, ratio6_1, true);
motor RM = motor(PORT20, ratio6_1, false);
motor LB = motor(PORT7, ratio6_1, true);
motor RB = motor(PORT2, ratio6_1, false);
motor intake = motor(PORT16, ratio18_1, true);
motor conveyorBelt = motor(PORT12, ratio18_1, false);
digital_out clamp1(Brain.ThreeWirePort.A);
digital_out doinker1(Brain.ThreeWirePort.B);
inertial Gyro1 = inertial(PORT13);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */

void stopDrive(){
  LB.stop(brake);
  RB.stop(brake);
  LM.stop(brake);
  RM.stop(brake);
}

void GyroPrint(){
  Brain.Screen.printAt(10, 20, "Heading = %0.2f", Gyro1.rotation(deg));
}

void mogoClamp(){ 
  clamp1.set(true);
}
 void mogoUnclamp() { 
  clamp1.set(false);
 }

void time_drive(int lspeed, int rspeed, float wt){
  LB.spin(fwd, lspeed, pct);
  RB.spin(fwd, rspeed, pct);
  LM.spin(fwd, lspeed, pct);
  RM.spin(fwd, rspeed, pct);
  wait(wt, msec);
}

double YOFFSET = 20;
void MotorDisplay(double y, double current, double temp){
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(5, YOFFSET + y, "Current: %0.1fA", current);
  if(current < 1)
    Brain.Screen.setFillColor(green);
    else if(current >= 1 && current <= 2.5)
      Brain.Screen.setFillColor(yellow);
      else
      Brain.Screen.setFillColor(red);
      Brain.Screen.drawRectangle(140, YOFFSET + y -15, 15, 15);

      Brain.Screen.setFillColor(transparent);
      Brain.Screen.printAt(160, YOFFSET + y, "Temp: %0.1fC", temp);
      if(temp < 45)
      Brain.Screen.setFillColor(green);
      else if(temp <= 50 && temp >= 45)
      Brain.Screen.setFillColor(yellow);
      else
      Brain.Screen.setFillColor(red);
      Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
      Brain.Screen.setFillColor(transparent);
}

void display(){
  double LMCurrent = LM.current(amp);
  double LMTemp = LM.temperature(celsius);
  double LBCurrent = LB.current(amp);
  double LBTemp = LB.temperature(celsius);
  double RMCurrent = RM.current(amp);
  double RMTemp = RM.temperature(celsius);
  double RBCurrent = RB.current(amp);
  double RBTemp = RB.temperature(celsius);

  if(LM.installed()){
    MotorDisplay(1, LMCurrent, LMTemp);
    Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");

  if(LB.installed()){
    MotorDisplay(31, LBCurrent, LBTemp);
    Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");

  if(RM.installed()){
    MotorDisplay(61, RMCurrent, RMTemp);
    Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");

  if(RB.installed()){
    MotorDisplay(91, RBCurrent, RBTemp);
    Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");
}

void PinchDrive(float target){
  // 1 inch off
  LM.setPosition(0, rev);
  float x = pi*LM.position(rev)*gr*Dia;
  float error = target - x;
  float kp = 3.0;
  float speed = kp*error;
  float accuracy = 0.5;
  while(fabs(error) > accuracy){
    time_drive(speed, speed, 25);
    x = pi*LM.position(rev)*gr*Dia;
    error = target - x;
    speed = kp*error;
  }
  stopDrive();
}

void GyroTurn(float target){
  float theta = 0.0;
  float error = target - theta;
  float accuracy = 0.5;
  float kp = 1.00; 
  float speed = kp*error;
  Gyro1.setRotation(0.0, deg);
  while(fabs(error) > accuracy){ 
    GyroPrint();
    time_drive(speed *(error)/error, -speed*(error)/error, 10);
    theta = Gyro1.rotation();
    error = target - theta;
    speed = kp*error;
  }
  stopDrive();

}


/*---------------------------------------------------------------------------*/

void pre_auton(void) {
//Gyro1.calibrate();


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  while(Gyro1.isCalibrating()){                                  
    wait(200, msec);
    // :)
  }
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
// ..........................................................................

// GyroTurn(-90);
// Brain.Screen.printAt(10, 50, "Heading = %0.2f", Gyro1.rotation(deg));
// GyroTurn testing
/*GyroTurn(90);
PinchDrive(48);*/

// PinchDrive testing
//PinchDrive(3);

// auto skills (alliance stakes)
/*mogoUnclamp();
PinchDrive(-3);
mogoClamp();
wait(250, msec);
conveyorBelt.spin(fwd, 100, pct);
GyroTurn(120);
PinchDrive(48);
GyroTurn(130);
PinchDrive(-35);
mogoUnclamp();
PinchDrive(20);
GyroTurn(-90);
PinchDrive(-100);
mogoClamp();
wait(250, msec);
GyroTurn(180);
PinchDrive(40);
GyroTurn(-90);=09
PinchDrive(40);
mogoUnclamp();*/

// auto skills (normal/no alliance stakes) not finished
//PinchDrive(-5);


// auto skills from 1st comp day
mogoUnclamp();
PinchDrive(-27);
mogoClamp();
time_drive(50, -50, 230);
PinchDrive(-30);

}

  // ..........................................................................

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void){
  // User control code here, inside the loop
  while (1) {
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main(){
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
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
motor ladybrown = motor(PORT4, ratio18_1, true);
motor ladybrown2 = motor(PORT8, ratio18_1, false);
digital_out clamp1(Brain.ThreeWirePort.A);
digital_out doinker1(Brain.ThreeWirePort.B);
inertial Gyro1 = inertial(PORT13);
rotation rotation1 = rotation(PORT17, true);

float armRotations[] = {0.0, -10.0, -90.0};
int currentIndex = 0;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */

// Auton Selector (GUI)
int AutonSelected = 0;
int AutonMin = 0;
int AutonMax = 1;

void drawGUI(){
	// Draws 2 buttons to be used for selecting auto
	Brain.Screen.clearScreen();
	Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
	Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
	Brain.Screen.setFillColor(red);
	Brain.Screen.drawRectangle(20, 50, 100, 100);
	Brain.Screen.drawCircle(300, 75, 25);
	Brain.Screen.printAt(25, 75, "Select");
	Brain.Screen.setFillColor(green);
	Brain.Screen.drawRectangle(170, 50, 100, 100);
	Brain.Screen.printAt(175, 75, "GO");
	Brain.Screen.setFillColor(black);
}

void selectAuton() {
		bool selectingAuton = true;
		
		int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
		int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
		
		// check to see if buttons were pressed
		if (x >= 20 && x <= 120 && y >= 50 && y <= 150){ // select button pressed
      AutonSelected++;
      if (AutonSelected > AutonMax){
        AutonSelected = AutonMin; // rollover
      }
      Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
		}
		
		
		if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
				selectingAuton = false; // GO button pressed
				Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonSelected);
		}
		
		if (!selectingAuton) {
				Brain.Screen.setFillColor(green);
				Brain.Screen.drawCircle(300, 75, 25);
		} else {
				Brain.Screen.setFillColor(red);
				Brain.Screen.drawCircle(300, 75, 25);
		}
		
		wait(10, msec); // slow it down
		Brain.Screen.setFillColor(black);
}

void stopDrive(){
  LB.stop(brake);
  RB.stop(brake);
  LM.stop(brake);
  RM.stop(brake);
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
  double IntakeTemp = intake.temperature(celsius);
  double IntakeCurrent = intake.current(amp);
  double ladyBrown2Current = ladybrown2.current(amp);
  double ladyBrown2Temp = ladybrown2.temperature(celsius);
  double ladyBrownCurrent = ladybrown.current(amp);
  double ladyBrownTemp = ladybrown.temperature(celsius);

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

  if(intake.installed()){
    MotorDisplay(121, IntakeCurrent, IntakeTemp);
    Brain.Screen.printAt(300, YOFFSET + 121, "Intake");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 121, "Intake Problem");

  if(ladybrown.installed()){
    MotorDisplay(151, ladyBrownCurrent, ladyBrownTemp);
    Brain.Screen.printAt(300, YOFFSET + 151, "LadyBrown1");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 151, "LadyBrown1 Problem");

  if(ladybrown2.installed()){
    MotorDisplay(181, ladyBrown2Current, ladyBrown2Temp);
    Brain.Screen.printAt(300, YOFFSET + 181, "LadyBrown2");
  }
  else
  Brain.Screen.printAt(5, YOFFSET + 181, "LadyBrown2 Problem");
}

void PinchDrive(float target){
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
  float accuracy = 1.75;
  float kp = 0.75;
  float speed = kp*error;
  Gyro1.setRotation(0.0, deg);
  while(fabs(error) >= accuracy){
    speed = kp*error;
    time_drive(speed, -speed, 10);
    theta = Gyro1.rotation();
    error = target - theta;
  }
  stopDrive();
}

/*int amountofstates = 3;
int currState = 0;
int ladybrowntarget = 0;
void nextState(){
  currState += 1;
  if(currState == 0){
    ladybrowntarget = 0;
  }
  else if(currState == 1){
    ladybrowntarget = 10;
  }
  else{
    ladybrowntarget = 90;
  }
}

void ladybrown_turn(){
  rotation1.resetPosition();
  float kp = 0.5;
  float initial = rotation1.angle(deg);
  float x = initial;
  float accuracy = ladybrowntarget + 2.0;
  while(x < accuracy or x > accuracy){
      float changed = rotation1.position(deg);
      x = changed - initial;
      float error = ladybrowntarget - x;
      float speed = kp*error;
      ladybrown.spin(reverse, speed, pct);
      ladybrown2.spin(reverse, speed, pct);
      wait(10, msec);
      ladybrown.stop(brake);
      ladybrown2.stop(brake);
      x = rotation1.angle(deg);
      error = ladybrowntarget - x;
      speed = kp*error;
      }
    ladybrown.stop(brake);
    ladybrown2.stop(brake);
  }*/

void armRotationControl(float target){
  float position = 0;
  float accuracy = 0.1; //change if needed
  float kp = 3.0; //change if needed
  float error = target;
  float speed = 0;
  rotation1.resetPosition();
  while(fabs(error) > accuracy){
    position = rotation1.position(deg);
    error = target - position;
    speed = error * kp;
    if(speed > 100) speed = 100;
    if(speed < -100) speed = -100;
    ladybrown.spin(fwd, speed, pct);
    ladybrown2.spin(fwd, speed, pct);
  }
}

/*---------------------------------------------------------------------------*/

void pre_auton(void) {
Gyro1.calibrate();


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  while(Gyro1.isCalibrating()){
    wait(10, msec);
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

  // testing 
  /*GyroTurn(90);
  wait(200, msec);
  PinchDrive(24); 
*/
  // 15-sec autonomous
 /*mogoUnclamp();
  PinchDrive(-27);
  mogoClamp();
  wait(250, msec);
  intake.spin(fwd, 100, pct);
  conveyorBelt.spin(fwd, 100, pct);
  wait(1.5, sec);
  conveyorBelt.spinToPosition(220, deg);   
  wait(0.5, sec);
  GyroTurn(190);
  wait(200, msec);
  PinchDrive(22);*/
   

  // touch bar autonomous
 /*mogoUnclamp();
  PinchDrive(-30);
  mogoClamp();
  wait(250, msec);
  intake.spin(fwd, 100, pct);
  conveyorBelt.spin(fwd, 100, pct);
  wait(1.5, sec);
  GyroTurn(100);
  wait(300, msec);
 PinchDrive(23.5);
  intake.spin(fwd, 0, pct);
  conveyorBelt.spin(fwd, 0, pct);*/
/*
  intake.stop(brake);
  conveyorBelt.stop(brake);*/
  /*wait(300, msec);
   GyroTurn(172.5);
   PinchDrive(43);
  conveyorBelt.spin(fwd, 100, pct);*/
  /*wait(300,msec);
  GyroTurn(130);
  PinchDrive(40);*/
   switch (AutonSelected) {
    case 0:
      //code 0 - left side passive
      mogoUnclamp();
      PinchDrive(-30);
      mogoClamp();
      wait(250, msec);
      intake.spin(fwd, 100, pct);
      conveyorBelt.spin(fwd, 100, pct);
      wait(1.5, sec);
      GyroTurn(100);
      wait(300, msec);
      PinchDrive(23.5);
      intake.spin(fwd, 0, pct);
      conveyorBelt.spin(fwd, 0, pct);
      wait(300, msec);
      GyroTurn(172.5);
      PinchDrive(43);
      conveyorBelt.spin(fwd, 100, pct);
      break;
				
    case 1:
      //code 2 - right side passive  mogoUnclamp();
      mogoUnclamp();
      PinchDrive(-30);
      mogoClamp();
      wait(250, msec);
      intake.spin(fwd, 100, pct);
      conveyorBelt.spin(fwd, 100, pct);
      wait(1.5, sec);
      GyroTurn(-100);
      wait(600, msec);
      PinchDrive(23.5);
      intake.stop(brake);
      conveyorBelt.stop(brake);
      wait(450, msec);
      GyroTurn(172.5);
      PinchDrive(43);
      conveyorBelt.spin(fwd, 100, pct);
    break;
  }

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
  bool lastButtonPress = false;
  while (1) {

      //drive
      display();
      Brain.Screen.printAt(10, 211, "Rotation: %0.2f", rotation1.position(deg));
      // drive
      int LeftJoystick = Controller1.Axis3.position(pct);
      int RightJoystick = Controller1.Axis2.position(pct);

      time_drive(LeftJoystick, RightJoystick, 10);

      // intake & conveyor belt
      if(Controller1.ButtonR1.PRESSED){
        intake.setVelocity(100, pct);
        intake.spin(fwd);
        conveyorBelt.setVelocity(100, pct);
        conveyorBelt.spin(fwd);
      }
      else if(Controller1.ButtonR2.PRESSED){
        intake.setVelocity(100, pct);
        intake.spin(reverse);
        conveyorBelt.setVelocity(100, pct);
        conveyorBelt.spin(reverse);
      }
      else if(Controller1.ButtonR1.RELEASED or Controller1.ButtonR2.RELEASED){
        intake.stop(brake);
        conveyorBelt.stop(brake);
      }

        // pneumatics (mobile goal clamp)
        if (Controller1.ButtonL2.PRESSED){
          mogoClamp();
        } 
        else if (Controller1.ButtonL1.PRESSED){
          mogoUnclamp();
        }

        if (Controller1.ButtonUp.PRESSED){
          doinker1.set(false);
        }
        else if (Controller1.ButtonDown.PRESSED){
          doinker1.set(true); 
        }

      // lady brown
      if(Controller1.ButtonX.pressing() && !lastButtonPress){
        currentIndex++;
        if(currentIndex >= sizeof(armRotations) / sizeof(armRotations[0])){
          currentIndex = 0;
        }
        armRotationControl(armRotations[currentIndex]);
      }
      lastButtonPress = Controller1.ButtonX.pressing();

        // delete comments up to here
        
        /*if(Controller1.ButtonL1.pressing()){
         clamp1.set(true);
      }
      else if(Controller1.ButtonL2.pressing()){
       clamp1.set(false);
      }
      */
    
     /* if(Controller1.ButtonX.PRESSED){
        nextState();
        ladybrown_turn(); */
      // lady brown
      /*if(Controller1.ButtonB.pressing()){
        ladybrown.spin(fwd, 100, pct);
        ladybrown2.spin(fwd, 100, pct);
      }
      else if(Controller1.ButtonX.pressing()){
        ladybrown.spin(reverse, 100, pct);
        ladybrown2.spin(reverse, 100, pct);
      }
      else if(Controller1.ButtonA.pressing()){
        rotation1.resetPosition();
        float angle = rotation1.angle(deg);
        ladybrown.spinToPosition(10, deg);
        ladybrown2.spinToPosition(10, deg);
        float x = rotation1.angle(deg);

       /* while(x > 10){
          ladybrown.spin
        }
      }*/
     /* else{
       ladybrown.stop(brake);
       ladybrown2.stop(brake);
      }
      } */
    
      /*if(Controller1.ButtonA.pressing()){
        ladybrown.spinToPosition(10, deg);
        ladybrown2.spinToPosition(10, deg);
      }
      else{
        ladybrown.stop(brake);
        ladybrown2.stop(brake);
      }*/
    

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wwasted resources
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